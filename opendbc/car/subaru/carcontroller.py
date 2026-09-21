import math
import numpy as np
from opendbc.can import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, make_tester_present_msg, rate_limit
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.lateral import apply_driver_steer_torque_limits, common_fault_avoidance
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.subaru import subarucan
from opendbc.car.subaru.values import DBC, GLOBAL_ES_ADDR, CanBus, CarControllerParams, SubaruFlags

from opendbc.sunnypilot.car.subaru.stop_and_go import SnGCarController

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
    self.lead_hold = 0
    self.braking = False
    self.decel_req = False
    self.crawl_floor = 0.0

    self.p = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint][Bus.pt])

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

    if CC.longActive:
      # The stock lookup has no speed term, so zero requested accel always commands
      # THROTTLE_INACTIVE, which only holds speed near 36-43 mph.
      v_ego_ff = CS.out.vEgo
      thr_hold = float(np.interp(v_ego_ff, self.p.THROTTLE_HOLD_BP, self.p.THROTTLE_HOLD_V))
      rpm_hold = float(np.interp(v_ego_ff, self.p.RPM_HOLD_BP, self.p.RPM_HOLD_V))
      thr_gain = float(np.interp(v_ego_ff, self.p.THROTTLE_GAIN_BP,
                                 self.p.THROTTLE_GAIN_V))
      accel = rate_limit(actuators.accel, self.accel_last, -ACCEL_RATE_LIMIT, ACCEL_RATE_LIMIT)
      self.accel_last = accel

      # One latched answer to "is a deceleration being requested", used by both protections below.
      # Keyed on the planner's request rather than accel_ff: uphill the grade term is positive and
      # would push accel_ff above the threshold, disabling both exactly where they are needed.
      if self.decel_req:
        self.decel_req = accel < self.p.DECEL_REQ_OFF
      else:
        self.decel_req = accel < self.p.DECEL_REQ_ON

      accel_ff = accel + accel_grade
      # Deceleration needs more counts per m/s^2 than acceleration, and unlike the up side it is
      # not speed dependent.
      thr_slope = thr_gain if accel_ff >= 0.0 else self.p.THROTTLE_DECEL_GAIN
      thr_raw = thr_hold + accel_ff * thr_slope
      # Near a stop the hold table holds the car against its own brake, so close the throttle
      # rather than part-closing it. Ramped out by 3 m/s, where delivery is already correct.
      if self.decel_req:
        keep = float(np.interp(v_ego_ff, self.p.THR_DECEL_CUT_BP,
                               self.p.THR_DECEL_CUT_V))
        thr_raw = CarControllerParams.THROTTLE_MIN + (thr_raw - CarControllerParams.THROTTLE_MIN) * keep
      apply_throttle = int(round(thr_raw))
      # The CVT ratio request carries real torque authority, so it gets the same treatment as the
      # throttle: an asymmetric gain and a slew limit, both bounded by what stock respects.
      rpm_gain = self.p.RPM_GAIN_UP if accel_ff >= 0.0 else self.p.RPM_GAIN_DOWN
      rpm_raw = rpm_hold + accel_ff * rpm_gain
      if self.rpm_last is None:
        self.rpm_last = rpm_hold
      apply_rpm = int(round(rate_limit(rpm_raw, self.rpm_last,
                                       -self.p.RPM_RATE_DOWN * DT_CTRL,
                                       self.p.RPM_RATE_UP * DT_CTRL)))
      self.rpm_last = apply_rpm

      # The brake supplies only what the throttle cannot. THR_DECEL is what a FULLY closed
      # throttle delivers, so the credit is scaled by how far the throttle is actually closed - a
      # throttle near its hold value is not slowing the car and must not be credited as if it
      # were. accel_ff carries the grade term, so a descent spills more to the brake, as
      # ford/carcontroller.py also does.
      thr_span = max(thr_hold - CarControllerParams.THROTTLE_ENGINE_BRAKE, 1.0)
      thr_closed = float(np.clip((thr_hold - apply_throttle) / thr_span, 0.0, 1.0))
      accel_thr_min = float(np.interp(v_ego_ff, self.p.THR_DECEL_BP,
                                      self.p.THR_DECEL_V)) * thr_closed
      brake_accel = min(0.0, accel_ff - accel_thr_min)
      apply_brake = int(round(np.interp(brake_accel,
                                        CarControllerParams.BRAKE_LOOKUP_BP, CarControllerParams.BRAKE_LOOKUP_V)))

      # Do not dribble the brake: anything under the deadband is drag, not deceleration. The
      # threshold is hysteretic because a demand parked on a single one toggles the brake, and
      # with it the brake lights, every few frames.
      threshold = self.p.BRAKE_DEADBAND_RELEASE if self.braking else self.p.BRAKE_DEADBAND
      if apply_brake < threshold:
        apply_brake = 0
      self.braking = apply_brake > 0
      # At a crawl the brake map, fitted at speed, under-asks badly against the torque converter.
      # Floor it while a deceleration is requested so the car finishes the stop instead of
      # creeping, ramped in by speed and by time so there is no step. Same gate as the throttle
      # cut: converter creep does not care about the grade.
      crawl_target = float(np.interp(v_ego_ff, self.p.CRAWL_BRAKE_BP,
                                     self.p.CRAWL_BRAKE_V)) if self.decel_req else 0.0
      step = self.p.CRAWL_BRAKE_RATE * DT_CTRL
      self.crawl_floor = rate_limit(crawl_target, self.crawl_floor, -step, step)
      apply_brake = max(apply_brake, int(round(self.crawl_floor)))

      # The upper limit is speed dependent for the same reason the panda's is: a flat count is a
      # different acceleration at every speed. Clip here rather than let the panda refuse the
      # frame - it rejects rather than clips, so an over-limit command is dropped entirely and the
      # car loses longitudinal for that cycle.
      thr_ceiling = min(float(np.interp(v_ego_ff, self.p.THROTTLE_MAX_BP,
                                        self.p.THROTTLE_MAX_V)),
                        CarControllerParams.THROTTLE_MAX)
      cruise_throttle = np.clip(apply_throttle, CarControllerParams.THROTTLE_MIN, thr_ceiling)
      cruise_rpm = np.clip(apply_rpm, CarControllerParams.RPM_MIN, CarControllerParams.RPM_MAX)
      cruise_brake = np.clip(apply_brake, CarControllerParams.BRAKE_MIN, CarControllerParams.BRAKE_MAX)
    else:
      self.accel_last = 0.0
      self.braking = False
      self.decel_req = False
      self.crawl_floor = 0.0
      self.rpm_last = None      # so the slew limit starts from the hold value, not a stale command
      cruise_throttle = CarControllerParams.THROTTLE_INACTIVE
      cruise_rpm = CarControllerParams.RPM_MIN
      cruise_brake = CarControllerParams.BRAKE_MIN

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
                                                      self.CP.openpilotLongitudinalControl, CC.longActive, cruise_rpm))

          can_sends.append(subarucan.create_es_brake(self.packer, self.frame // 5, CS.es_brake_msg, bus,
                                                     self.CP.openpilotLongitudinalControl, CC.longActive, cruise_brake))

          can_sends.append(subarucan.create_es_distance(self.packer, self.frame // 5, CS.es_distance_msg, bus, pcm_cancel_cmd,
                                                        self.CP.openpilotLongitudinalControl, cruise_brake > 0, cruise_throttle))
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
