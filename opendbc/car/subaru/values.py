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

  # Throttle and CVT RPM required to hold speed. Fitted from 29k engaged samples. kp is 0 on this
  # plant, so the integrator is the whole feedback path and an error here does not show up as a
  # tracking error - it becomes a permanent standing offset in the integrator.
  "THROTTLE_HOLD_BP": [0.0, 3.0, 5.0, 9.0, 13.0, 15.0, 17.0, 19.0, 21.0, 25.0, 29.0, 32.5],
  "THROTTLE_HOLD_V": [1818, 1870, 1920, 1950, 2030, 2110, 2350, 2380, 2520, 2660, 2870, 3020],
  # Counts per m/s^2 for acceleration, speed indexed: CVT gearing makes the car far more
  # responsive at low speed, by a factor of 1.75 across the range. Regressed from
  # throttle-above-hold against achieved accel over 46k engaged samples at the measured 0.60 s lag.
  "THROTTLE_GAIN_BP": [1.0, 3.0, 5.5,  8.0, 11.0, 15.5, 21.5],  # m/s
  "THROTTLE_GAIN_V": [400, 510, 550,  610,  800,  690,  690],
  # The fit above used acceleration samples only and says nothing about the way down, so the
  # decel side is a separate constant rather than inheriting the speed curve.
  "THROTTLE_DECEL_GAIN": 1005,
  # Fraction of the throttle-above-idle kept while a deceleration is requested. Below about 1 m/s
  # the hold table and the torque converter together drive the car forward against its own brake,
  # delivering 17% of the request. Ramped out entirely by 3 m/s, where delivery is already 83-85%.
  "THR_DECEL_CUT_BP": [1.0, 3.0],  # m/s
  "THR_DECEL_CUT_V": [0.0, 1.0],
  # Deceleration the closed throttle makes on its own, per speed. Measured from 3822 coastdown
  # samples - driver off both pedals, cruise and openpilot long off - which is the only way to
  # observe it: this controller never coasts, since the brake engages the moment the throttle
  # floors, so no closed-throttle zero-brake sample exists in its own data. Gravity is removed by
  # ADDING g*sin(pitch), because aEgo carries -g*sin (slope of aEgo on g*sin(pitch) = -1.04).
  #   v m/s     1     3     5     7     9    12    16    20    24    28
  #   a       +.21  -.24  -.28  -.33  -.40  -.44  -.44  -.52  -.55  -.62
  # The old table asserted -0.65 from 5 m/s up. Engine braking does not reach that until ~27 m/s,
  # so between 5 and 20 m/s the brake was being credited up to 0.37 m/s^2 the throttle never
  # produced, and 171 counts per m/s^2 of brake was withheld - worst in the 11-25 mph band where a
  # stop is approached. That is the same double-count defect as before, in the other direction.
  "THR_DECEL_BP": [0.0,  1.0,  3.0,   5.0,   7.0,   9.0,   12.0,  16.0,  20.0,  24.0,  28.0],
  "THR_DECEL_V": [0.25, 0.21, -0.24, -0.28, -0.33, -0.40, -0.44, -0.44, -0.52, -0.55, -0.62],
  # Re-measured 2026-09-15 against what the stock camera commands at steady cruise. The five flat
  # 2034 entries were 515-624 rpm HIGH at 18-26 m/s (n >= 112 stock frames at every point from
  # 15 m/s up), which is plant gain the integrator then has to fight - the mid-speed abruptness.
  # The dip at 20 relative to 16 is real CVT behaviour, not a typo: the ratio talls out as speed
  # rises. Values BELOW 15 m/s are deliberately UNCHANGED - EyeSight's ACC was never active below
  # 5.86 m/s in 438 segments, so there is no stock support down there, and the proposed low-speed
  # raise would have pushed +0.29 to +0.40 m/s^2 of extra forward authority into exactly the
  # stop-and-go regime where the car already creeps into leads.
  # NOT stock-derived, deliberately. Stock EyeSight holds this speed at roughly 1350-1710 rpm over
  # 20-28 m/s and these values sit 350-700 above that. The stock figures were measured and tried,
  # and acceleration delivery collapsed from 0.85x to 0.23x at 45-70 mph.
  #
  # The reason is that a hold table and its gain table are identified only as a PAIR: the hold sets
  # the equilibrium, the gain sets the counts per m/s^2 above it, and the car's response depends on
  # both. These values are the ones the measured THROTTLE_GAIN_V and RPM_GAIN_UP were fitted
  # against. Moving a hold table toward stock while leaving the gains alone takes authority out of
  # both channels at once.
  #
  # So when retuning a model: refit hold AND gain together, from the same frames, and check
  # delivered acceleration by speed band afterwards - holding speed correctly is necessary but not
  # sufficient. Score delivery against longitudinalPlan.aTarget, never against actuators.accel,
  # which carries the integrator's standing offset and will read as a collapse that is not there.
  "RPM_HOLD_BP": [0.0, 8.0, 10.0, 16.0, 20.0, 22.0, 24.0, 26.0, 28.0, 30.0, 32.5],
  "RPM_HOLD_V": [600,  600,  901, 1486, 2020, 2034, 2034, 2034, 2034, 2198, 2198],
  # Asymmetric: the car needs more ratio to accelerate than to give back. Scored sample by sample
  # against stock's own command, and corroborated by stock's median margin over hold per accel bin.
  "RPM_GAIN_UP": 1500,
  "RPM_GAIN_DOWN": 700,
  # The DOWN limit stops the ratio request collapsing in a step; stock respects it 99% of the time
  # and never exceeds it while decelerating hard, so it cannot blunt a real deceleration. The UP
  # limit is loose enough to catch only genuine discontinuities, well above the natural slew.
  "RPM_RATE_UP": 2000.0,  # counts per second
  "RPM_RATE_DOWN": 400.0,
  # EyeSight commands 0 rather than a handful of counts, and the car's Brake_Status feedback
  # confirms the hydraulics really do actuate on 1-10 count commands, so those are genuine drag.
  # Zeroing below this takes engaged brake duty from 65.7% to 26.5% with no loss of authority
  # anywhere it matters - the median brake at -2.0 m/s^2 is unchanged.
  "BRAKE_DEADBAND": 30,
  # Release lower than engage, so a steady small request stays on. With a single threshold the
  # brake chatters whenever the demand sits on it - 16.6 applications a minute, median 0.25 s, many
  # of them crossing the brake-light threshold - which is most of gentle downhill braking.
  "BRAKE_DEADBAND_RELEASE": 12,
  # Below walking pace the torque converter creeps the car forward and the brake map, fitted at
  # speed, asks for far too little. Applied as a floor under a decel request, never an addition, so
  # it cannot stack with the demand above it. Sized from what the stock long build sends while
  # completing its own stops (85-136 counts); at 100 the car still rolled the last few feet,
  # delivering 0.69x of the request below 3.4 mph against 1.07x above 6.7.
  "CRAWL_BRAKE_BP": [0.8, 1.5],  # m/s, ramped out so there is no step at the threshold
  "CRAWL_BRAKE_V": [130.0, 0.0],
  # ...and ramped in over time as well as speed. Applying the floor as a step put the full value on
  # the car in one frame, which is the "lurch" at the end of a stop. At this rate the floor above
  # takes 0.39 s to reach, still quick enough to close the gap the brake map leaves at a crawl.
  "CRAWL_BRAKE_RATE": 130.0 / 0.39,  # counts per second: the floor above, reached in 0.39 s

  # Whether a deceleration was requested. Hysteretic for the same reason the brake deadband is: a
  # single threshold chatters whenever the request hovers on it, and each toggle hands the throttle
  # back and drops the crawl floor, so the car creeps forward mid-stop. The two thresholds are far
  # apart because the signals they separate are: a request hovering around zero peaks near
  # +0.1 m/s^2, while a genuine pull-away passes +0.40 within one frame of the car moving.
  "DECEL_REQ_ON": -0.05,  # m/s^2, engage
  "DECEL_REQ_OFF": 0.35,  # m/s^2, release

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
  # What the car is told to hold once LongCtrlState.stopping latches. This is the shared default
  # from interfaces.py rather than an independently fitted number, set explicitly here so it is
  # visible and tunable per model instead of silently inherited. Validated on the Crosstrek:
  # -2.01 commanded at a standstill, 407 brake counts, and the car holds. It is a plant property
  # in principle - mass and torque-converter creep decide how hard a car must be held against its
  # own idle - so a heavier model may want more. Peers differ widely: vw -0.55, honda bosch -4.0.
  "STOP_ACCEL": -2.0,                   # m/s^2
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

  THROTTLE_INACTIVE = 1818  # corresponds to zero acceleration
  THROTTLE_ENGINE_BRAKE = 808  # while braking, eyesight sets throttle to this, probably for engine braking

  BRAKE_MIN = 0
  BRAKE_MAX = 600  # about -3.5m/s2 from testing
  BRAKE_LIGHTS_THRESHOLD = 70  # brake command at which the lamps, and the cluster's drawing of them, light

  RPM_MIN = 0
  RPM_MAX = 3600

  BRAKE_LOOKUP_BP = [-3.5, 0]
  BRAKE_LOOKUP_V = [BRAKE_MAX, BRAKE_MIN]


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
