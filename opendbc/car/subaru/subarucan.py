from opendbc.car import structs
from opendbc.car.subaru.values import CanBus, CarControllerParams

VisualAlert = structs.CarControl.HUDControl.VisualAlert


def create_steering_control(packer, apply_torque, steer_req):
  values = {
    "LKAS_Output": apply_torque,
    "LKAS_Request": steer_req,
    "SET_1": 1
  }
  return packer.make_can_msg("ES_LKAS", 0, values)


def create_steering_control_angle(packer, apply_torque, steer_req):
  values = {
    "LKAS_Output": apply_torque,
    "LKAS_Request": steer_req,
    "SET_3": 3
  }
  return packer.make_can_msg("ES_LKAS_ANGLE", 0, values)


def create_steering_status(packer):
  return packer.make_can_msg("ES_LKAS_State", 0, {})


def create_es_distance(packer, frame, es_distance_msg, bus, pcm_cancel_cmd, long_enabled = False, brake_cmd = False, cruise_throttle = 0):
  values = {s: es_distance_msg[s] for s in [
    "CHECKSUM",
    "Signal1",
    "Cruise_Fault",
    "Cruise_Throttle",
    "Signal2",
    "Car_Follow",
    "Low_Speed_Follow",
    "Cruise_Soft_Disable",
    "Signal7",
    "Cruise_Brake_Active",
    "Distance_Swap",
    "Cruise_EPB",
    "Signal4",
    "Close_Distance",
    "Signal5",
    "Cruise_Cancel",
    "Cruise_Set",
    "Cruise_Resume",
    "Signal6",
  ]}

  values["COUNTER"] = frame % 0x10

  if long_enabled:
    values["Cruise_Throttle"] = cruise_throttle

    # Do not disable openpilot on Eyesight Soft Disable, if openpilot is controlling long
    values["Cruise_Soft_Disable"] = 0
    values["Cruise_Fault"] = 0

    values["Cruise_Brake_Active"] = brake_cmd

  if pcm_cancel_cmd:
    values["Cruise_Cancel"] = 1
    values["Cruise_Throttle"] = 1818 # inactive throttle

  return packer.make_can_msg("ES_Distance", bus, values)


def create_es_lkas_state(packer, frame, es_lkas_state_msg, enabled, lat_active, cruise_available, dash_indicators,
                         long_active, standstill, visual_alert, left_line, right_line,
                         left_lane_depart, right_lane_depart):
  values = {s: es_lkas_state_msg[s] for s in [
    "CHECKSUM",
    "LKAS_Alert_Msg",
    "Signal1",
    "LKAS_ACTIVE",
    "LKAS_Dash_State",
    "Signal2",
    "Backward_Speed_Limit_Menu",
    "LKAS_Left_Line_Enable",
    "LKAS_Left_Line_Light_Blink",
    "LKAS_Right_Line_Enable",
    "LKAS_Right_Line_Light_Blink",
    "LKAS_Left_Line_Visible",
    "LKAS_Right_Line_Visible",
    "LKAS_Alert",
    "Signal3",
  ]}

  values["COUNTER"] = frame % 0x10

  # Filter the stock LKAS "Keep hands on wheel" alert
  if values["LKAS_Alert_Msg"] == 1:
    values["LKAS_Alert_Msg"] = 0

  # Filter the stock LKAS sending an audible alert when it turns off LKAS
  if values["LKAS_Alert"] == 27:
    values["LKAS_Alert"] = 0

  # Filter the stock LKAS sending an audible alert when "Keep hands on wheel" alert is active (2020+ models)
  if values["LKAS_Alert"] == 28 and values["LKAS_Alert_Msg"] == 7:
    values["LKAS_Alert"] = 0

  # Filter the stock LKAS sending an audible alert when "Keep hands on wheel OFF" alert is active (2020+ models)
  if values["LKAS_Alert"] == 30:
    values["LKAS_Alert"] = 0

  # The camera runs its own ACC state machine and will not hold at a standstill for more than a
  # few seconds before beeping for the driver to take the brake. openpilot holds on brake pressure
  # for as long as it needs to, so the demand is spurious: it fired 2 s after standstill and
  # toggled three times in one second, which is what the driver heard. Gated on the hold, so an
  # Audio_Beep raised for any other reason still reaches the driver.
  if values["LKAS_Alert"] == 24 and long_active and standstill:
    values["LKAS_Alert"] = 0

  # Filter the stock LKAS sending "Keep hands on wheel OFF" alert (2020+ models)
  if values["LKAS_Alert_Msg"] == 7:
    values["LKAS_Alert_Msg"] = 0

  # Show Keep hands on wheel alert for openpilot steerRequired alert
  if visual_alert == VisualAlert.steerRequired:
    values["LKAS_Alert_Msg"] = 1

  # Ensure we don't overwrite potentially more important alerts from stock (e.g. FCW)
  if visual_alert == VisualAlert.ldw and values["LKAS_Alert"] == 0:
    if left_lane_depart:
      values["LKAS_Alert"] = 12  # Left lane departure dash alert
    elif right_lane_depart:
      values["LKAS_Alert"] = 11  # Right lane departure dash alert

  # Obstacle Detected, flashing red with repeated beeps. After the ldw branch so a collision
  # warning wins, and on the cluster because ES_Infotainment only reaches the head unit.
  if dash_indicators and visual_alert == VisualAlert.fcw:
    values["LKAS_Alert"] = 2

  if enabled:
    values["LKAS_ACTIVE"] = 1  # Show LKAS lane lines
    if dash_indicators:
      # The cluster draws no line at all unless Enable is set, and stock holds it at 0 the whole
      # time openpilot is steering, so the colour below never reaches the screen without this.
      values["LKAS_Left_Line_Enable"] = 1
      values["LKAS_Right_Line_Enable"] = 1

  if dash_indicators:
    # 2 = green, 1 = white, 0 = off. White whenever cruise is available but openpilot is not
    # steering, mirroring Cruise_Disengaged_Dash, so the two indicators agree.
    values["LKAS_Dash_State"] = 2 if lat_active else 1 if cruise_available else 0
    # 0 = grey, 1 = white, 2 = green. Green only while actually steering, as nissan does.
    line_color = 2 if lat_active else 1 if enabled else 0
    values["LKAS_Left_Line_Visible"] = line_color if left_line else 0
    values["LKAS_Right_Line_Visible"] = line_color if right_line else 0
  else:
    values["LKAS_Dash_State"] = 2 if enabled else 0
    values["LKAS_Left_Line_Visible"] = int(left_line)
    values["LKAS_Right_Line_Visible"] = int(right_line)

  return packer.make_can_msg("ES_LKAS_State", CanBus.main, values)


def create_es_dashstatus(packer, frame, dashstatus_msg, enabled, long_enabled, long_active, dash_indicators,
                         lead_visible, distance_bars, brake_value, brake_pressed, standstill):
  values = {s: dashstatus_msg[s] for s in [
    "CHECKSUM",
    "PCB_Off",
    "LDW_Off",
    "Signal1",
    "Cruise_State_Msg",
    "LKAS_State_Msg",
    "Signal2",
    "Cruise_Soft_Disable",
    "Cruise_Status_Msg",
    "Signal3",
    "Cruise_Distance",
    "Signal4",
    "Conventional_Cruise",
    "Signal5",
    "Cruise_Disengaged_Dash",
    "Cruise_Activated_Dash",
    "Signal6",
    "Cruise_Set_Speed",
    "Cruise_Fault",
    "Cruise_On",
    "Display_Own_Car",
    "Brake_Lights",
    "Car_Follow",
    "Signal7",
    "Far_Distance",
    "Cruise_State",
  ]}

  values["COUNTER"] = frame % 0x10

  if long_enabled:
    values["Car_Follow"] = int(lead_visible)

    values["PCB_Off"] = 1 # AEB is not preserved, so show the PCB_Off on dash
    values["LDW_Off"] = 0
    values["Cruise_Fault"] = 0

    if dash_indicators:
      # Bitfield: bit0 is HOLD. 3 would add READY, which the cluster renders as "Ready Hold".
      values["Cruise_State"] = 1 if (long_active and standstill) else 0
      # Green while openpilot has the gas and brakes, white whenever cruise main is on but it
      # does not, which covers the gas override.
      values["Cruise_Activated_Dash"] = int(long_active)
      values["Cruise_Disengaged_Dash"] = int(values["Cruise_On"] and not long_active)

      # Cluster follow-distance bars, the stock personality readout. 1-4, 0 blanks them.
      values["Cruise_Distance"] = distance_bars

      # Draws the brake lights on the car image, on both the cluster and the MFD. Stock only
      # reports the driver's own braking about two thirds of the time, so OR it in directly.
      values["Brake_Lights"] = int(values["Brake_Lights"] or brake_pressed or
                                   brake_value >= CarControllerParams.BRAKE_LIGHTS_THRESHOLD)

      # Both latch: after ~5s the crossed-out EyeSight mark replaces the lead car and the
      # distance bars, which openpilot now owns.
      values["Cruise_Soft_Disable"] = 0
      values["Cruise_Status_Msg"] = 0
    else:
      values["Cruise_State"] = 0
      # TODO: Cruise_Activated_dash should respect gas pressed and standstill stock behavior
      values["Cruise_Activated_Dash"] = enabled
      values["Cruise_Disengaged_Dash"] = 0

  # Filter stock LKAS disabled and Keep hands on steering wheel OFF alerts
  if values["LKAS_State_Msg"] in (2, 3):
    values["LKAS_State_Msg"] = 0

  return packer.make_can_msg("ES_DashStatus", CanBus.main, values)


def create_es_brake(packer, frame, es_brake_msg, bus, long_enabled, long_active, brake_value):
  values = {s: es_brake_msg[s] for s in [
    "CHECKSUM",
    "Signal1",
    "Brake_Pressure",
    "AEB_Status",
    "Cruise_Brake_Lights",
    "Cruise_Brake_Fault",
    "Cruise_Brake_Active",
    "Cruise_Activated",
    "Signal3",
  ]}

  values["COUNTER"] = frame % 0x10

  if long_enabled:
    # openpilot sends the only ES_Brake the brake ECU sees, so forwarding the camera's AEB
    # status alongside our own Brake_Pressure would emit a frame stock never produces.
    values["AEB_Status"] = 0
    values["Cruise_Brake_Fault"] = 0
    values["Cruise_Activated"] = long_active

    values["Brake_Pressure"] = brake_value

    values["Cruise_Brake_Active"] = brake_value > 0
    values["Cruise_Brake_Lights"] = brake_value >= CarControllerParams.BRAKE_LIGHTS_THRESHOLD

  return packer.make_can_msg("ES_Brake", bus, values)


def create_es_status(packer, frame, es_status_msg, bus, long_enabled, long_active, cruise_rpm):
  values = {s: es_status_msg[s] for s in [
    "CHECKSUM",
    "Signal1",
    "Cruise_Fault",
    "Cruise_RPM",
    "Cruise_Activated",
    "Brake_Lights",
    "Cruise_Hold",
    "Signal3",
  ]}

  values["COUNTER"] = frame % 0x10

  if long_enabled:
    values["Cruise_RPM"] = cruise_rpm
    values["Cruise_Fault"] = 0

    values["Cruise_Activated"] = long_active

  return packer.make_can_msg("ES_Status", bus, values)


def create_es_infotainment(packer, frame, es_infotainment_msg, visual_alert):
  # Filter stock LKAS disabled and Keep hands on steering wheel OFF alerts
  values = {s: es_infotainment_msg[s] for s in [
    "CHECKSUM",
    "LKAS_State_Infotainment",
    "LKAS_Blue_Lines",
    "Signal1",
    "Signal2",
  ]}

  values["COUNTER"] = frame % 0x10

  if values["LKAS_State_Infotainment"] in (3, 4):
    values["LKAS_State_Infotainment"] = 0

  # Show Keep hands on wheel alert for openpilot steerRequired alert
  if visual_alert == VisualAlert.steerRequired:
    values["LKAS_State_Infotainment"] = 3

  # Show Obstacle Detected for fcw
  if visual_alert == VisualAlert.fcw:
    values["LKAS_State_Infotainment"] = 2

  return packer.make_can_msg("ES_Infotainment", CanBus.main, values)


def create_es_highbeamassist(packer):
  values = {
    "HBA_Available": False,
  }

  return packer.make_can_msg("ES_HighBeamAssist", CanBus.main, values)


def create_es_static_1(packer):
  values = {
    "SET_3": 3,
  }

  return packer.make_can_msg("ES_STATIC_1", CanBus.main, values)


def create_es_static_2(packer):
  values = {
    "SET_3": 3,
  }

  return packer.make_can_msg("ES_STATIC_2", CanBus.main, values)


# *** Subaru Pre-global ***

def subaru_preglobal_checksum(packer, values, addr, checksum_byte=7):
  dat = packer.make_can_msg(addr, 0, values)[1]
  return (sum(dat[:checksum_byte]) + sum(dat[checksum_byte+1:])) % 256


def create_preglobal_steering_control(packer, frame, apply_torque, steer_req):
  values = {
    "COUNTER": frame % 0x08,
    "LKAS_Command": apply_torque,
    "LKAS_Active": steer_req,
  }
  values["Checksum"] = subaru_preglobal_checksum(packer, values, "ES_LKAS")

  return packer.make_can_msg("ES_LKAS", CanBus.main, values)


def create_preglobal_es_distance(packer, cruise_button, es_distance_msg):
  values = {s: es_distance_msg[s] for s in [
    "Cruise_Throttle",
    "Signal1",
    "Car_Follow",
    "Signal2",
    "Cruise_Brake_Active",
    "Distance_Swap",
    "Standstill",
    "Signal3",
    "Close_Distance",
    "Signal4",
    "Standstill_2",
    "Cruise_Fault",
    "Signal5",
    "COUNTER",
    "Signal6",
    "Cruise_Button",
    "Signal7",
  ]}

  values["Cruise_Button"] = cruise_button
  values["Checksum"] = subaru_preglobal_checksum(packer, values, "ES_Distance")

  return packer.make_can_msg("ES_Distance", CanBus.main, values)


def subaru_checksum(address: int, sig, d: bytearray) -> int:
  s = 0
  addr = address
  while addr:
    s += addr & 0xFF
    addr >>= 8
  for i in range(1, len(d)):
    s += d[i]
  return s & 0xFF
