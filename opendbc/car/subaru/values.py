from dataclasses import dataclass, field
from enum import Enum, IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarFootnote, CarHarness, CarDocs, CarParts, Column
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries, p16

Ecu = CarParams.Ecu


# ---------------------------------------------------------------------------------------------
# Longitudinal tuning, per model.
#
# Every number here was measured on a 2021 Crosstrek Sport (SUBARU_IMPREZA_2020). These are plant
# properties - mass, CVT calibration, aero, torque-converter creep - so they do NOT transfer
# between models: an Ascent is some 600 kg heavier than a Crosstrek and will want its own tables.
# Each supported platform gets its own copy, and the copies start identical, so a model can be
# retuned from tester data in isolation without disturbing the one that is known good.
#
# UNMEASURED platforms are marked below. Until a platform has its own data it runs Crosstrek
# numbers, which is one reason this stays alpha-longitudinal.
#
# One entry is not free to diverge upward: THROTTLE_MAX_V is mirrored by the panda in
# SUBARU_MAX_GAS_LOOKUP (opendbc/safety/modes/subaru.h), and the panda is handed safety flags,
# not a fingerprint - it enforces ONE curve for every Subaru. A model may clip below that freely;
# a model needing MORE counts for the same acceleration cannot be served without raising the
# shared ceiling for everyone, which is a safety-model change rather than a retune.
# ---------------------------------------------------------------------------------------------
_CROSSTREK_LONG: dict = {
  # The per-car lever on how briskly the car answers a speed-up request. 0.6 railed 17.2% of the
  # time below 30 mph; 1.2 cleared that above 30 but still railed 36.3% at 10-20 mph and 18.7% at
  # 20-30. Read off longitudinalPlan, which this does not clip, the demand peaked at 1.52 over a
  # whole drive and never passed 1.43 in either railing band - it is already bounded by the
  # planner's own A_CRUISE_MAX_VALS, which peaks at 1.6. Matching that stops the per-car value
  # shadowing the shared envelope, and stays inside the throttle channel's own speed-dependent
  # ceiling of 2.0 m/s^2 - see THROTTLE_MAX_BP / THROTTLE_MAX_V.
  "ACCEL_MAX": 1.6,  # m/s^2
  # The count that yields 2.0 m/s^2 at each speed, which is the same curve the panda enforces in
  # SUBARU_MAX_GAS_LOOKUP (opendbc/safety/modes/subaru.h). A flat ceiling cannot bound acceleration
  # here, because the throttle needed merely to hold a speed rises with it, so the headroom above
  # hold - the part that accelerates - shrinks as speed rises: the old flat 3400 allowed +3.96
  # m/s^2 from rest and only +0.77 at 65 mph, which is why hills felt dead.
  # The panda evaluates this same curve one m/s further along, so a command clipped here is always
  # inside what the safety model will accept rather than sitting exactly on its edge.
  "THROTTLE_MAX_BP": [0.0,  15.0, 29.0],  # m/s
  "THROTTLE_MAX_V": [2618, 3514, 4250],  # counts

  # Deceleration a shut throttle makes on its own, gravity-free; positive at a crawl, where converter
  # creep pushes the car. From coastdowns, within 0.05 m/s^2 of the camera's own zero-brake frames.
  "A_COAST_BP": [0.0,  1.0,  3.0,   5.0,   7.0,   9.0,   12.0,  16.0,  20.0,  24.0,  28.0],  # m/s
  "A_COAST_V": [0.25, 0.21, -0.24, -0.28, -0.33, -0.40, -0.44, -0.44, -0.52, -0.55, -0.62],
  # How much less the open branch's floor decelerates than a shut throttle, which is where the brake's
  # zero sits while the throttle is open. 0.128 +/- 0.012 over 192 steady runs on four routes, flat
  # over 14-35 m/s.
  "HANDOFF_BIAS": 0.13,  # m/s^2
  # Share of it applied, by speed. None below 2 m/s: the engine idles at either throttle position
  # there, so there is no step to bridge and the bias would only put brake under a positive request.
  "HANDOFF_TAPER_BP": [2.0, 3.0],  # m/s
  "HANDOFF_TAPER_V": [0.0, 1.0],
  # Delays the shut until the brake under the open throttle decelerates the car more than a shut
  # throttle would, so the two positions overlap and leave no band the integral-only loop hunts
  # across. Small brake commands do little, so that takes 68 counts, still under the lamp threshold.
  # Never delays the re-open: above A_COAST a shut throttle cannot meet the request.
  "HANDOFF_HYST": 0.24,  # m/s^2
  # How long a shut throttle's extra engine braking takes to build (all of it by 1.8 s on the
  # camera's own shuts), and an opened throttle's torque to come back (its 0.54 s lag).
  "HANDOFF_SHUT_TIME": 1.8,  # s
  "HANDOFF_OPEN_TIME": 0.5,  # s

  # What the camera holds a speed with, and counts per m/s^2 above that, fitted to its own commands.
  # Hold and gain are identified as a pair: retune them together, and score delivery against
  # longitudinalPlan.aTarget rather than actuators.accel. Below 2 m/s even the floor holds a speed or
  # more, so there the pair is the line through what the camera's commands deliver: a hold under 1818.
  "THROTTLE_HOLD_BP": [0.3,   1.8,  3.4,  5.7,  8.3, 11.5, 14.5, 17.8, 20.5, 23.3, 26.4, 29.9, 33.2],
  "THROTTLE_HOLD_V": [1792, 1840, 1959, 2141, 2141, 2242, 2309, 2369, 2468, 2642, 2769, 2797, 3025],
  "THROTTLE_GAIN_BP": [0.3,  1.8,  3.4,  5.7,  8.3, 11.5, 14.5, 17.8, 20.5, 23.3, 26.4, 29.9, 33.2],
  "THROTTLE_GAIN_V": [458,  502,  403,  426,  512,  667,  872, 1105, 1233, 1450, 1457, 1457, 1727],
  "THROTTLE_CHORD_MIN": 0.15,  # m/s^2, the least request a chord up from the open floor spans
  # The shut's band where there is no bias to bridge, below 2 m/s: under BRAKE_DEADBAND / BRAKE_GAIN, so the open throttle
  # never brakes, and wide enough that a crawl's request, which rests on A_COAST, cannot flip the throttle every frame.
  "CRAWL_HYST": 0.06,  # m/s^2
  # The ratio request tracks the throttle, so it is fitted command on command.
  "RPM_HOLD_BP": [0.0,  0.5,  1.1,  1.8,  2.6,  5.7,  8.3, 11.5, 14.5, 17.8, 20.5, 23.3, 26.4, 29.9, 33.2],
  "RPM_HOLD_V": [100,  336,  578,  907, 1061, 1094, 1202, 1221, 1265, 1349, 1435, 1592, 1756, 2006, 2146],
  "RPM_PER_THROTTLE": 0.64,
  # What the camera requests with the throttle shut.
  "RPM_COAST_BP": [0.0, 0.22, 0.52, 1.11, 1.84, 2.58, 5.09, 11.32, 19.54, 23.87, 28.9],
  "RPM_COAST_V": [100,  207,  336,  578,  907, 1061, 1073,  1108,  1128,  1369,  1614],
  "RPM_STANDSTILL": 100,  # at a standstill, as the camera does
  # The DOWN limit stops the ratio request collapsing in a step; stock respects it 99% of the time
  # and never exceeds it while decelerating hard, so it cannot blunt a real deceleration. The UP
  # limit is loose enough to catch only genuine discontinuities, well above the natural slew.
  "RPM_RATE_UP": 2000.0,  # counts per second
  "RPM_RATE_DOWN": 400.0,

  "BRAKE_GAIN": 185.0,  # counts per m/s^2 below the brake's zero, fitted to the camera's command
  # Smaller commands are drag, not deceleration. Kept under HANDOFF_BIAS * BRAKE_GAIN, 24 counts, so
  # the brake is on before the request reaches A_COAST, and released only at zero: aEgo noise moves
  # the request by more than a release band of a few counts, and each crossing would be a dab.
  "BRAKE_DEADBAND": 12,
  "BRAKE_DEADBAND_RELEASE": 1,

  # Set on CarParams in interface.py rather than read by the controller, but they are plant
  # properties like everything else here, so they belong with the model's tables.
  #
  # Lag-scanned by correlating throttle-above-hold against achieved accel; the peak is at 0.60 s
  # (r=0.76), far from the 0.15 default. Get this wrong on a new model and the loop destabilises -
  # kp is 0, so the integrator is the only thing holding it together.
  "LONGITUDINAL_ACTUATOR_DELAY": 0.6,   # s
  # Closes the loop: the port commands throttle counts with no car-side feedback, so without ki
  # any mapping error becomes a permanent speed offset. Deliberately low - the hold table already
  # removes most of the bias, leaving the integrator to trim, and 0.6 s of delay plus a CVT makes
  # a large ki the fast route to a limit cycle. ford uses 0.5, honda nidec 1.2/0.8/0.5.
  "KI_BP": [0., 5., 35.],               # m/s
  "KI_V": [0.25, 0.25, 0.2],
  # The request ramps toward this once LongCtrlState.stopping latches. With the grade term that
  # holds 314 - 185 * grade counts on a descent, against EyeSight's 324 - 174 * grade.
  "STOP_ACCEL": -1.45,                  # m/s^2
  # The floor under it: EyeSight settles at 304 on the flat and uphill alike, and the Crosstrek has a
  # manual parking brake, so brake pressure is the only hold there is.
  "HOLD_BRAKE": 304.0,  # counts
  # Ramped rather than stepped, which is the lurch at the end of a stop: gently where the request
  # already holds the car, quickly on a steep climb, where the grade term cancels it and only the
  # floor holds. Indexed by accel_grade as computed, pitch calibration residual included.
  "HOLD_BRAKE_RATE_BP": [0.7, 1.2],  # m/s^2 of grade
  "HOLD_BRAKE_RATE_V": [150.0, 500.0],  # counts per second, applying
  "HOLD_BRAKE_RELEASE": 1000.0,  # counts per second, releasing
}


class CarControllerParams:
  def __init__(self, CP):
    self.STEER_STEP = 2                # how often we update the steer cmd
    self.STEER_DELTA_UP = 50           # torque increase per refresh, 0.8s to max
    self.STEER_DELTA_DOWN = 70         # torque decrease per refresh
    self.STEER_DRIVER_ALLOWANCE = 60   # allowed driver torque before start limiting
    self.STEER_DRIVER_MULTIPLIER = 50  # weight driver torque heavily
    self.STEER_DRIVER_FACTOR = 1       # from dbc

    if CP.flags & SubaruFlags.GLOBAL_GEN2:
      self.STEER_MAX = 1500
      self.STEER_DELTA_UP = 35
      self.STEER_DELTA_DOWN = 50
    elif CP.carFingerprint == CAR.SUBARU_IMPREZA_2020:
      self.STEER_DELTA_UP = 35
      self.STEER_MAX = 1439
    else:
      self.STEER_MAX = 2047

    # Longitudinal tables resolve per model; see LONG_TUNE below. Cars that cannot enable
    # openpilot longitudinal build this class for the lateral limits above and never read them.
    for _k, _v in long_tune(CP.carFingerprint).items():
      setattr(self, _k, _v)

  # Not a comfort knob - it bounds lead-following and emergency braking too, so it is shared
  # across models and stays at the opendbc default.
  ACCEL_MIN = -3.5  # m/s^2

  THROTTLE_MIN = 808
  # Absolute ceiling, at the top of what stock EyeSight itself commands. The speed-indexed
  # THROTTLE_MAX_BP / THROTTLE_MAX_V binds below it almost everywhere.
  THROTTLE_MAX = 4100

  # The inactive command, and the bottom of the open branch: the camera sends either THROTTLE_MIN
  # or at least this, never anything in between.
  THROTTLE_INACTIVE = 1818

  BRAKE_MIN = 0
  BRAKE_MAX = 600  # A_COAST - 3.24 m/s^2 at BRAKE_GAIN
  # The lamps, and the cluster's drawing of them, follow the camera's rule, so a brake hovering near one count cannot flash them.
  BRAKE_LAMP_ON = 80
  BRAKE_LAMP_OFF = 20
  BRAKE_LAMP_DELAY = 0.4  # s
  # Above a crawl the camera keeps them dark while its braking barely slows the car, as when holding speed downhill.
  BRAKE_LAMP_DECEL_ON = -0.7  # m/s^2 of aEgo to light them
  BRAKE_LAMP_DECEL_OFF = -0.4  # m/s^2 to keep them lit
  # Below these, to light them and to keep them lit, the counts alone decide: a band, so a speed held near it cannot flicker them.
  BRAKE_LAMP_DECEL_V = (5.0, 6.0)  # m/s
  # ES_Distance.Cruise_Brake_Active follows the brake past the camera's 20-count resting notch, split
  # around it because this controller's request does not rest on a notch and would dither across it.
  BRAKE_TIER2_ON = 24
  BRAKE_TIER2_OFF = 12

  RPM_MIN = 0
  RPM_MAX = 3600


class SubaruSafetyFlags(IntFlag):
  GEN2 = 1
  LONG = 2
  PREGLOBAL_REVERSED_DRIVER_TORQUE = 4


class SubaruFlags(IntFlag):
  # Detected flags
  SEND_INFOTAINMENT = 1
  DISABLE_EYESIGHT = 2

  # Static flags
  GLOBAL_GEN2 = 4

  # Cars that temporarily fault when steering angle rate is greater than some threshold.
  # Appears to be all torque-based cars produced around 2019 - present
  STEER_RATE_LIMITED = 8
  PREGLOBAL = 16
  HYBRID = 32
  LKAS_ANGLE = 64

  # Cluster and MFD indicators driven from openpilot's own state rather than passed through from
  # the camera. Global gen1 shares the cluster vocabulary, so all four platforms set it. A
  # per-platform flag rather than a gen1 test, so a car whose cluster renders a value differently
  # can be dropped on its own.
  DASH_INDICATORS = 128


GLOBAL_ES_ADDR = 0x787
GEN2_ES_BUTTONS_DID = b'\x11\x30'


class CanBus:
  main = 0
  alt = 1
  camera = 2


class Footnote(Enum):
  GLOBAL = CarFootnote(
    "In the non-US market, openpilot requires the car to come equipped with EyeSight with Lane Keep Assistance.",
    Column.PACKAGE)
  EXP_LONG = CarFootnote(
    "Enabling longitudinal control (alpha) will disable all EyeSight functionality, including AEB, LDW, and RAB.",
    Column.LONGITUDINAL)


@dataclass
class SubaruCarDocs(CarDocs):
  package: str = "EyeSight Driver Assistance"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.subaru_a]))
  footnotes: list[Enum] = field(default_factory=lambda: [Footnote.GLOBAL])

  def init_make(self, CP: CarParams):
    if CP.alphaLongitudinalAvailable:
      self.footnotes.append(Footnote.EXP_LONG)


@dataclass
class SubaruPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'subaru_global_2017_generated'})

  def init(self):
    if self.flags & SubaruFlags.HYBRID:
      self.dbc_dict = {Bus.pt: 'subaru_global_2020_hybrid_generated'}


@dataclass
class SubaruGen2PlatformConfig(SubaruPlatformConfig):
  def init(self):
    super().init()
    self.flags |= SubaruFlags.GLOBAL_GEN2
    if not (self.flags & SubaruFlags.LKAS_ANGLE):
      self.flags |= SubaruFlags.STEER_RATE_LIMITED


class CAR(Platforms):
  # Global platform
  SUBARU_ASCENT = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Ascent 2019-21", "All")],
    CarSpecs(mass=2031, wheelbase=2.89, steerRatio=13.5),
    flags=SubaruFlags.DASH_INDICATORS,
  )
  SUBARU_OUTBACK = SubaruGen2PlatformConfig(
    [SubaruCarDocs("Subaru Outback 2020-22", "All", car_parts=CarParts.common([CarHarness.subaru_b]))],
    CarSpecs(mass=1568, wheelbase=2.67, steerRatio=17),
  )
  SUBARU_LEGACY = SubaruGen2PlatformConfig(
    [SubaruCarDocs("Subaru Legacy 2020-22", "All", car_parts=CarParts.common([CarHarness.subaru_b]))],
    SUBARU_OUTBACK.specs,
  )
  SUBARU_IMPREZA = SubaruPlatformConfig(
    [
      SubaruCarDocs("Subaru Impreza 2017-19"),
      SubaruCarDocs("Subaru Crosstrek 2018-19", video="https://youtu.be/Agww7oE1k-s?t=26"),
      SubaruCarDocs("Subaru XV 2018-19", video="https://youtu.be/Agww7oE1k-s?t=26"),
    ],
    CarSpecs(mass=1568, wheelbase=2.67, steerRatio=15),
    flags=SubaruFlags.DASH_INDICATORS,
  )
  SUBARU_IMPREZA_2020 = SubaruPlatformConfig(
    [
      SubaruCarDocs("Subaru Impreza 2020-22"),
      SubaruCarDocs("Subaru Crosstrek 2020-23"),
      SubaruCarDocs("Subaru XV 2020-21"),
    ],
    CarSpecs(mass=1480, wheelbase=2.67, steerRatio=17),
    flags=SubaruFlags.STEER_RATE_LIMITED | SubaruFlags.DASH_INDICATORS,
  )
  # TODO: is there an XV and Impreza too?
  SUBARU_CROSSTREK_HYBRID = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Crosstrek Hybrid 2020", car_parts=CarParts.common([CarHarness.subaru_b]))],
    CarSpecs(mass=1668, wheelbase=2.67, steerRatio=17),
    flags=SubaruFlags.HYBRID,
  )
  SUBARU_FORESTER = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Forester 2019-21", "All")],
    CarSpecs(mass=1568, wheelbase=2.67, steerRatio=17),
    flags=SubaruFlags.STEER_RATE_LIMITED | SubaruFlags.DASH_INDICATORS,
  )
  SUBARU_FORESTER_HYBRID = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Forester Hybrid 2020")],
    SUBARU_FORESTER.specs,
    flags=SubaruFlags.HYBRID,
  )
  # Pre-global
  SUBARU_FORESTER_PREGLOBAL = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Forester 2017-18")],
    CarSpecs(mass=1568, wheelbase=2.67, steerRatio=20),
    {Bus.pt: 'subaru_forester_2017_generated'},
    flags=SubaruFlags.PREGLOBAL,
  )
  SUBARU_LEGACY_PREGLOBAL = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Legacy 2015-18")],
    CarSpecs(mass=1568, wheelbase=2.67, steerRatio=12.5),
    {Bus.pt: 'subaru_outback_2015_generated'},
    flags=SubaruFlags.PREGLOBAL,
  )
  SUBARU_OUTBACK_PREGLOBAL = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Outback 2015-17")],
    SUBARU_FORESTER_PREGLOBAL.specs,
    {Bus.pt: 'subaru_outback_2015_generated'},
    flags=SubaruFlags.PREGLOBAL,
  )
  SUBARU_OUTBACK_PREGLOBAL_2018 = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Outback 2018-19")],
    SUBARU_FORESTER_PREGLOBAL.specs,
    {Bus.pt: 'subaru_outback_2019_generated'},
    flags=SubaruFlags.PREGLOBAL,
  )
  # Angle LKAS
  SUBARU_FORESTER_2022 = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Forester 2022-24", "All", car_parts=CarParts.common([CarHarness.subaru_c]))],
    SUBARU_FORESTER.specs,
    flags=SubaruFlags.LKAS_ANGLE,
  )
  SUBARU_OUTBACK_2023 = SubaruGen2PlatformConfig(
    [SubaruCarDocs("Subaru Outback 2023", "All", car_parts=CarParts.common([CarHarness.subaru_d]))],
    SUBARU_OUTBACK.specs,
    flags=SubaruFlags.LKAS_ANGLE,
  )
  SUBARU_ASCENT_2023 = SubaruGen2PlatformConfig(
    [SubaruCarDocs("Subaru Ascent 2023", "All", car_parts=CarParts.common([CarHarness.subaru_d]))],
    SUBARU_ASCENT.specs,
    flags=SubaruFlags.LKAS_ANGLE,
  )

LONG_TUNE: dict = {
  CAR.SUBARU_IMPREZA_2020: dict(_CROSSTREK_LONG),   # measured, 2021 Crosstrek Sport
  CAR.SUBARU_IMPREZA:      dict(_CROSSTREK_LONG),   # UNMEASURED - inherited
  CAR.SUBARU_FORESTER:     dict(_CROSSTREK_LONG),   # UNMEASURED - inherited
  CAR.SUBARU_ASCENT:       dict(_CROSSTREK_LONG),   # UNMEASURED - inherited
}


def long_tune(candidate) -> dict:
  """The longitudinal tables for a platform.

  Cars that cannot enable openpilot longitudinal - preglobal, gen2, hybrid, angle-LKAS - still
  build CarParams and CarControllerParams for the lateral limits, so this falls back rather than
  raising. They never read a value from it."""
  return LONG_TUNE.get(candidate, _CROSSTREK_LONG)


SUBARU_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.APPLICATION_DATA_IDENTIFICATION)
SUBARU_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.APPLICATION_DATA_IDENTIFICATION)

# The EyeSight ECU takes 10s to respond to SUBARU_VERSION_REQUEST properly,
# log this alternate manufacturer-specific query
SUBARU_ALT_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(0xf100)
SUBARU_ALT_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + \
  p16(0xf100)

FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"(?:[\x00-\xff]{4,5}|[\x00-\xff]{8}|[\x00-\xff]{10})",
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, SUBARU_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, SUBARU_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.abs, Ecu.eps, Ecu.fwdCamera, Ecu.engine, Ecu.transmission],
      logging=True,
    ),
    # Non-OBD requests
    # Some Eyesight modules fail on TESTER_PRESENT_REQUEST
    # TODO: check if this resolves the fingerprinting issue for the 2023 Ascent and other new Subaru cars
    Request(
      [SUBARU_VERSION_REQUEST],
      [SUBARU_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.fwdCamera],
      bus=0,
    ),
    Request(
      [SUBARU_ALT_VERSION_REQUEST],
      [SUBARU_ALT_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.fwdCamera],
      bus=0,
      logging=True,
    ),
    Request(
      [StdQueries.DEFAULT_DIAGNOSTIC_REQUEST, StdQueries.TESTER_PRESENT_REQUEST, SUBARU_VERSION_REQUEST],
      [StdQueries.DEFAULT_DIAGNOSTIC_RESPONSE, StdQueries.TESTER_PRESENT_RESPONSE, SUBARU_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.fwdCamera],
      bus=0,
      logging=True,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, SUBARU_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, SUBARU_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.abs, Ecu.eps, Ecu.fwdCamera, Ecu.engine, Ecu.transmission],
      bus=0,
    ),
    # GEN2 powertrain bus query
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, SUBARU_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, SUBARU_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.abs, Ecu.eps, Ecu.fwdCamera, Ecu.engine, Ecu.transmission],
      bus=1,
      obd_multiplexing=False,
    ),
  ],
  # We don't get the EPS from non-OBD queries on GEN2 cars. Note that we still attempt to match when it exists
  non_essential_ecus={
    Ecu.eps: [c for c in CAR if c.config.flags & SubaruFlags.GLOBAL_GEN2],
  }
)

DBC = CAR.create_dbc_map()
