import math
import numpy as np
from opendbc.can import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, make_tester_present_msg, rate_limit, structs
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.lateral import apply_driver_steer_torque_limits, common_fault_avoidance
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.subaru import subarucan
from opendbc.car.subaru.values import DBC, GLOBAL_ES_ADDR, CanBus, CarControllerParams, SubaruFlags

from opendbc.sunnypilot.car.subaru.stop_and_go import SnGCarController

LongCtrlState = structs.CarControl.Actuators.LongControlState

# FIXME: These limits aren't exact. The real limit is more than likely over a larger time period and
# involves the total steering angle change rather than rate, but these limits work well for now
MAX_STEER_RATE = 25  # deg/s
MAX_STEER_RATE_FRAMES = 7  # tx control frames needed before torque can be cut

# Gravity is uncompensated by the speed-only feedforward, so the coefficient is 1.0. The clip is
# a backstop against a bad pose estimate; it also trims grades steeper than about 9 deg.
GRADE_FF_GAIN = 1.0
GRADE_FF_MAX = 1.5  # m/s^2

# Backstop against steps in the actuator command, same value toyota uses. It sits above the
# planner's own jerk budget, so it shapes nothing in normal driving and only catches
# discontinuities such as the longActive rising edge.
ACCEL_RATE_LIMIT = 4.0 * DT_CTRL  # m/s^2 per frame

# drive_helpers.should_stop's thresholds: a stopping state above the speed is not a standstill to
# hold, and a request below the acceleration is not a request to move off.
VEGO_STOPPING = 0.3  # m/s
ACCEL_GO = 0.1  # m/s^2
# Resuming this slowly after an override, a car that has stood still since it moved off may be
# rolling back. That standstill can come just before the override, so it is tracked in both states.
VEGO_RESUME_HOLD = 2.0  # m/s
RESUME_HOLD_FRAMES = int(1.0 / DT_CTRL)
# Where gravity beats creep, the throttle takes over under the hold as it lets go. Released first, the car rolls back
# past VEGO_STOPPING, which reads as moving off and ends the stopping state for good.
HILL_START_MARGIN = 0.3  # m/s^2 of grade term beyond A_COAST at rest

# The model drops a distant lead for a few tenths of a second at a time, which flickers the
# cluster's lead icon. Dash only - the control path still sees the raw signal.
LEAD_HOLD_FRAMES = int(1.0 / DT_CTRL)


class CarController(CarControllerBase, SnGCarController):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    SnGCarController.__init__(self, CP, CP_SP)
    self.apply_torque_last = 0

    self.cruise_button_prev = 0
    self.steer_rate_counter = 0

    # Raw pitch is noisy enough to chatter the throttle; grade changes slowly.
    self.pitch = FirstOrderFilter(0.0, 0.5, DT_CTRL)

    self.accel_last = 0.0
    self.rpm_last = None
    self.hold_latched = False
    self.resume_frames = 0
    self.stood_still = False
    self.lead_hold = 0
    self.braking = False
    self.coasting = False
    self.shut_frac = 0.0
    self.stop_hold = 0.0

    self.p = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint][Bus.pt])

    self.brake_tier2 = False

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel

    can_sends = []

    # Track pitch every cycle, not just while engaged, so the filter is converged at engagement.
    # Same as toyota/carcontroller.py.
    if len(CC.orientationNED) == 3:
      self.pitch.update(CC.orientationNED[1])
    accel_grade = float(np.clip(GRADE_FF_GAIN * math.sin(self.pitch.x) * ACCELERATION_DUE_TO_GRAVITY,
                                -GRADE_FF_MAX, GRADE_FF_MAX))

    dash_indicators = bool(self.CP.flags & SubaruFlags.DASH_INDICATORS)
    if hud_control.leadVisible:
      self.lead_hold = LEAD_HOLD_FRAMES
    elif self.lead_hold > 0:
      self.lead_hold -= 1
    lead_visible = hud_control.leadVisible or (dash_indicators and self.lead_hold > 0)

    # *** steering ***
    if (self.frame % self.p.STEER_STEP) == 0:
      apply_torque = int(round(actuators.torque * self.p.STEER_MAX))

      # limits due to driver torque

      new_torque = int(round(apply_torque))
      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.p)

      if not CC.latActive:
        apply_torque = 0

      if self.CP.flags & SubaruFlags.PREGLOBAL:
        can_sends.append(subarucan.create_preglobal_steering_control(self.packer, self.frame // self.p.STEER_STEP, apply_torque, CC.latActive))
      else:
        apply_steer_req = CC.latActive

        if self.CP.flags & SubaruFlags.STEER_RATE_LIMITED:
          # Steering rate fault prevention
          self.steer_rate_counter, apply_steer_req = \
            common_fault_avoidance(abs(CS.out.steeringRateDeg) > MAX_STEER_RATE, apply_steer_req,
                                   self.steer_rate_counter, MAX_STEER_RATE_FRAMES)

        can_sends.append(subarucan.create_steering_control(self.packer, apply_torque, apply_steer_req))

      self.apply_torque_last = apply_torque

    # *** longitudinal ***

    if CS.out.standstill:
      self.stood_still = True
    elif CS.out.vEgo >= VEGO_RESUME_HOLD or (CC.longActive and actuators.accel >= ACCEL_GO):
      self.stood_still = False

    if CC.longActive:
      v_ego_ff = CS.out.vEgo
      hill_start = (self.stop_hold > 0.0 and not self.hold_latched and actuators.accel >= 0.0 and
                    accel_grade > float(np.interp(0.0, self.p.A_COAST_BP, self.p.A_COAST_V)) + HILL_START_MARGIN)
      if hill_start:
        self.accel_last = actuators.accel
      accel = rate_limit(actuators.accel, self.accel_last, -ACCEL_RATE_LIMIT, ACCEL_RATE_LIMIT)
      self.accel_last = accel
      accel_ff = accel + accel_grade

      # The camera sends THROTTLE_MIN or at least THROTTLE_INACTIVE, never anything between, and the
      # open branch's floor decelerates HANDOFF_BIAS less than a shut throttle. So the brake starts at
      # that floor and the throttle shuts only once the brake carries the difference.
      a_coast = float(np.interp(v_ego_ff, self.p.A_COAST_BP, self.p.A_COAST_V))
      a_open_min = a_coast + self.p.HANDOFF_BIAS * float(np.interp(v_ego_ff, self.p.HANDOFF_TAPER_BP,
                                                                    self.p.HANDOFF_TAPER_V))
      if self.coasting:
        self.coasting = accel_ff < a_coast
      else:
        # With no bias to bridge, the camera brakes under a shut throttle.
        self.coasting = accel_ff < a_coast - (self.p.HANDOFF_HYST if a_open_min > a_coast else self.p.CRAWL_HYST)

      thr_hold = float(np.interp(v_ego_ff, self.p.THROTTLE_HOLD_BP, self.p.THROTTLE_HOLD_V))
      rpm_coast = float(np.interp(v_ego_ff, self.p.RPM_COAST_BP, self.p.RPM_COAST_V))
      if self.coasting:
        apply_throttle = float(CarControllerParams.THROTTLE_MIN)
        apply_rpm = rpm_coast
      elif accel_ff < a_open_min:
        # EyeSight reaches most of this band with the throttle still open and no brake, which this
        # map cannot, so it brakes lightly here and raises the brake annunciation where EyeSight
        # would not. At a crawl the floor is positive, converter creep, so a crawl is held on brake.
        apply_throttle = float(CarControllerParams.THROTTLE_INACTIVE)
      else:
        # A chord to the floor: the bottom few hundred counts are nearly flat, so a slope misses both ends.
        thr_gain = float(np.interp(v_ego_ff, self.p.THROTTLE_GAIN_BP, self.p.THROTTLE_GAIN_V))
        a_join = max(0.0, a_open_min + self.p.THROTTLE_CHORD_MIN)
        if accel_ff >= a_join:
          apply_throttle = thr_hold + accel_ff * thr_gain
        else:
          apply_throttle = float(np.interp(accel_ff, [a_open_min, a_join],
                                           [CarControllerParams.THROTTLE_INACTIVE, thr_hold + a_join * thr_gain]))

      if not self.coasting:
        # Below about 2200 counts the camera holds the ratio near RPM_COAST instead of tracking.
        rpm_hold = float(np.interp(v_ego_ff, self.p.RPM_HOLD_BP, self.p.RPM_HOLD_V))
        apply_rpm = max(rpm_hold + (apply_throttle - thr_hold) * self.p.RPM_PER_THROTTLE, rpm_coast)

      # The brake's zero follows the throttle's deceleration as it arrives, not as it is commanded,
      # so neither a shut nor a re-open steps the brake ahead of the car.
      self.shut_frac = rate_limit(float(self.coasting), self.shut_frac, -DT_CTRL / self.p.HANDOFF_OPEN_TIME,
                                  DT_CTRL / self.p.HANDOFF_SHUT_TIME)
      brake_zero = a_open_min + (a_coast - a_open_min) * self.shut_frac
      apply_brake = max(0.0, (brake_zero - accel_ff) * self.p.BRAKE_GAIN)

      # shouldStop never reaches CarControl, so the stopping state is the signal, as in gm, honda and
      # toyota. A floor, not a replacement, which would drop the brake while the hold ramps in.
      stopping = actuators.longControlState == LongCtrlState.stopping and v_ego_ff < VEGO_STOPPING
      if stopping and CS.out.standstill:
        self.hold_latched = True
      elif actuators.accel >= ACCEL_GO:
        # Once stopped, only a request to go ends it: wheel speed is unsigned, so a rollback reads as
        # moving off and would end the stopping state just when the car needs holding.
        self.hold_latched = False
      # After an override the planner restarts from aEgo, which reads a rollback as speeding up.
      if v_ego_ff >= VEGO_RESUME_HOLD:
        self.resume_frames = 0
      elif self.resume_frames > 0:
        self.resume_frames -= 1
        self.hold_latched = self.hold_latched or actuators.accel < 0.0
      hold = self.p.HOLD_BRAKE if stopping or self.hold_latched else 0.0
      hold_rate = float(np.interp(accel_grade, self.p.HOLD_BRAKE_RATE_BP, self.p.HOLD_BRAKE_RATE_V))
      self.stop_hold = rate_limit(hold, self.stop_hold, -self.p.HOLD_BRAKE_RELEASE * DT_CTRL, hold_rate * DT_CTRL)
      if hill_start and hold == 0.0:
        apply_brake = max(apply_brake, self.stop_hold)
      elif hold > 0.0 or self.stop_hold > 0.0:
        apply_throttle = float(CarControllerParams.THROTTLE_MIN)
        apply_rpm = float(self.p.RPM_STANDSTILL)
        apply_brake = max(apply_brake, self.stop_hold)
        self.coasting = True
        # The request that reproduces the brake now on the car, so leaving the hold neither steps
        # the throttle nor spends the launch unwinding the ramp to stopAccel as brake.
        self.accel_last = brake_zero - apply_brake / self.p.BRAKE_GAIN - accel_grade

      apply_brake = int(round(apply_brake))
      threshold = self.p.BRAKE_DEADBAND_RELEASE if self.braking else self.p.BRAKE_DEADBAND
      if apply_brake < threshold:
        apply_brake = 0
      self.braking = apply_brake > 0

      # The ratio request carries real torque authority, so it keeps a slew limit bounded by what
      # stock respects. Seeded from the map, so the first engaged frame does not ramp.
      if self.rpm_last is None:
        self.rpm_last = apply_rpm
      apply_rpm = rate_limit(apply_rpm, self.rpm_last, -self.p.RPM_RATE_DOWN * DT_CTRL,
                             self.p.RPM_RATE_UP * DT_CTRL)
      self.rpm_last = apply_rpm

      # The upper limit is speed dependent for the same reason the panda's is: a flat count is a
      # different acceleration at every speed. Clip here rather than let the panda refuse the
      # frame - it rejects rather than clips, so an over-limit command is dropped entirely and the
      # car loses longitudinal for that cycle.
      thr_ceiling = min(float(np.interp(v_ego_ff, self.p.THROTTLE_MAX_BP,
                                        self.p.THROTTLE_MAX_V)),
                        CarControllerParams.THROTTLE_MAX)
      cruise_throttle = np.clip(round(apply_throttle), CarControllerParams.THROTTLE_MIN, thr_ceiling)
      cruise_rpm = np.clip(round(apply_rpm), CarControllerParams.RPM_MIN, CarControllerParams.RPM_MAX)
      cruise_brake = np.clip(apply_brake, CarControllerParams.BRAKE_MIN, CarControllerParams.BRAKE_MAX)
    else:
      self.hold_latched = False
      self.resume_frames = RESUME_HOLD_FRAMES if self.stood_still else 0
      self.accel_last = 0.0
      self.braking = False
      self.coasting = False
      self.shut_frac = 0.0
      self.stop_hold = 0.0
      self.rpm_last = None      # so the slew limit starts from the map, not a stale command
      cruise_throttle = CarControllerParams.THROTTLE_INACTIVE
      cruise_rpm = CarControllerParams.RPM_MIN
      cruise_brake = CarControllerParams.BRAKE_MIN

    if self.brake_tier2:
      self.brake_tier2 = cruise_brake > CarControllerParams.BRAKE_TIER2_OFF
    else:
      self.brake_tier2 = cruise_brake > CarControllerParams.BRAKE_TIER2_ON

    # *** alerts and pcm cancel ***
    if self.CP.flags & SubaruFlags.PREGLOBAL:
      if self.frame % 5 == 0:
        # 1 = main, 2 = set shallow, 3 = set deep, 4 = resume shallow, 5 = resume deep
        # disengage ACC when OP is disengaged
        if pcm_cancel_cmd:
          cruise_button = 1
        # turn main on if off and past start-up state
        elif not CS.out.cruiseState.available and CS.ready:
          cruise_button = 1
        else:
          cruise_button = CS.cruise_button

        # unstick previous mocked button press
        if cruise_button == 1 and self.cruise_button_prev == 1:
          cruise_button = 0
        self.cruise_button_prev = cruise_button

        can_sends.append(subarucan.create_preglobal_es_distance(self.packer, cruise_button, CS.es_distance_msg))

    else:
      if self.frame % 10 == 0:
        can_sends.append(subarucan.create_es_dashstatus(self.packer, self.frame // 10, CS.es_dashstatus_msg,
                                                        CC.enabled, self.CP.openpilotLongitudinalControl, CC.longActive,
                                                        dash_indicators, lead_visible, hud_control.leadDistanceBars,
                                                        cruise_brake, CS.out.brakePressed, CS.out.standstill))

        can_sends.append(subarucan.create_es_lkas_state(self.packer, self.frame // 10, CS.es_lkas_state_msg, CC.enabled, CC.latActive,
                                                        CS.out.cruiseState.available, dash_indicators,
                                                        CC.longActive, CS.out.standstill, hud_control.visualAlert,
                                                        hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                                        hud_control.leftLaneDepart, hud_control.rightLaneDepart))

        if self.CP.flags & SubaruFlags.SEND_INFOTAINMENT:
          can_sends.append(subarucan.create_es_infotainment(self.packer, self.frame // 10, CS.es_infotainment_msg, hud_control.visualAlert))

      # gen2 moves the cruise messages to the alt bus, which is also where carstate reads them
      # back (carstate.py, cp_es_distance / cp_es_brake). The panda only permits them there for
      # gen2, so sending on main would have every longitudinal frame rejected.
      bus = CanBus.alt if self.CP.flags & SubaruFlags.GLOBAL_GEN2 else CanBus.main

      if self.CP.openpilotLongitudinalControl:
        if self.frame % 5 == 0:
          can_sends.append(subarucan.create_es_status(self.packer, self.frame // 5, CS.es_status_msg, bus,
                                                      self.CP.openpilotLongitudinalControl, CC.longActive, cruise_rpm,
                                                      cruise_brake > 0))

          can_sends.append(subarucan.create_es_brake(self.packer, self.frame // 5, CS.es_brake_msg, bus,
                                                     self.CP.openpilotLongitudinalControl, CC.longActive, cruise_brake))

          can_sends.append(subarucan.create_es_distance(self.packer, self.frame // 5, CS.es_distance_msg, bus, pcm_cancel_cmd,
                                                        self.CP.openpilotLongitudinalControl, self.brake_tier2, cruise_throttle))
      else:
        if pcm_cancel_cmd:
          if not (self.CP.flags & SubaruFlags.HYBRID):
            can_sends.append(subarucan.create_es_distance(self.packer, CS.es_distance_msg["COUNTER"] + 1, CS.es_distance_msg, bus, pcm_cancel_cmd))

      if self.CP.flags & SubaruFlags.DISABLE_EYESIGHT:
        # Tester present (keeps eyesight disabled)
        if self.frame % 100 == 0:
          can_sends.append(make_tester_present_msg(GLOBAL_ES_ADDR, CanBus.camera, suppress_response=True))

        # Create all of the other eyesight messages to keep the rest of the car happy when eyesight is disabled
        if self.frame % 5 == 0:
          can_sends.append(subarucan.create_es_highbeamassist(self.packer))

        if self.frame % 10 == 0:
          can_sends.append(subarucan.create_es_static_1(self.packer))

        if self.frame % 2 == 0:
          can_sends.append(subarucan.create_es_static_2(self.packer))

    can_sends.extend(SnGCarController.create_stop_and_go(self, self.packer, CC, CS, self.frame))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.p.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends
