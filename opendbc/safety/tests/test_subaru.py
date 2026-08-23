#!/usr/bin/env python3
import enum
import itertools
import unittest
import numpy as np

from functools import partial

from opendbc.car.subaru.values import SubaruSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, round_speed, away_round


class SubaruMsg(enum.IntEnum):
  Brake_Status      = 0x13c
  CruiseControl     = 0x240
  Throttle          = 0x40
  Steering_Torque   = 0x119
  Wheel_Speeds      = 0x13a
  ES_LKAS           = 0x122
  ES_LKAS_ANGLE     = 0x124
  ES_Brake          = 0x220
  ES_Distance       = 0x221
  ES_Status         = 0x222
  ES_DashStatus     = 0x321
  ES_LKAS_State     = 0x322
  ES_Infotainment   = 0x323
  ES_LKAS_Alert     = 0x3C4
  ES_UDS_Request    = 0x787
  ES_HighBeamAssist = 0x22A
  ES_STATIC_1       = 0x325
  ES_STATIC_2       = 0x121
  Dashlights        = 0x390
  Comfort_Control   = 0x6bb


SUBARU_MAIN_BUS = 0
SUBARU_ALT_BUS  = 1
SUBARU_CAM_BUS  = 2


def lkas_tx_msgs(alt_bus, lkas_msg=SubaruMsg.ES_LKAS, lkas_alert=False):
  msgs = [[lkas_msg,                    SUBARU_MAIN_BUS],
          [SubaruMsg.ES_Distance,       alt_bus],
          [SubaruMsg.ES_DashStatus,     SUBARU_MAIN_BUS],
          [SubaruMsg.ES_LKAS_State,     SUBARU_MAIN_BUS],
          [SubaruMsg.ES_Infotainment,   SUBARU_MAIN_BUS]]
  if lkas_alert:
    msgs.append([SubaruMsg.ES_LKAS_Alert, SUBARU_MAIN_BUS])
  return msgs


def comfort_tx_msgs(alt_bus):
  return [[SubaruMsg.Comfort_Control,    alt_bus],
          [SubaruMsg.Dashlights,         alt_bus]]


def long_tx_msgs(alt_bus):
  return [[SubaruMsg.ES_Brake,          alt_bus],
          [SubaruMsg.ES_Status,         alt_bus]]


def gen2_long_additional_tx_msgs():
  return [[SubaruMsg.ES_UDS_Request,    SUBARU_CAM_BUS],
          [SubaruMsg.ES_HighBeamAssist, SUBARU_MAIN_BUS],
          [SubaruMsg.ES_STATIC_1,       SUBARU_MAIN_BUS],
          [SubaruMsg.ES_STATIC_2,       SUBARU_MAIN_BUS]]


def fwd_blacklisted_addr(lkas_msg=SubaruMsg.ES_LKAS, lkas_alert=False):
  addrs = [lkas_msg, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State, SubaruMsg.ES_Infotainment]
  if lkas_alert:
    addrs.append(SubaruMsg.ES_LKAS_Alert)
  return {SUBARU_CAM_BUS: addrs}


# LKAS_ANGLE cars additionally replace the camera's duplicate alert message
ANGLE_RELAY_MALFUNCTION_ADDRS = {SUBARU_MAIN_BUS: (SubaruMsg.ES_LKAS_ANGLE, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                                                   SubaruMsg.ES_Infotainment, SubaruMsg.ES_LKAS_Alert)}


class TestSubaruSafetyBase(common.CarSafetyTest):
  FLAGS = 0
  RELAY_MALFUNCTION_ADDRS = {SUBARU_MAIN_BUS: (SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                                               SubaruMsg.ES_Infotainment)}
  FWD_BLACKLISTED_ADDRS = fwd_blacklisted_addr()

  MAX_RT_DELTA = 940

  DRIVER_TORQUE_ALLOWANCE = 60
  DRIVER_TORQUE_FACTOR = 50

  ALT_MAIN_BUS = SUBARU_MAIN_BUS
  ALT_CAM_BUS = SUBARU_CAM_BUS

  DEG_TO_CAN = 100

  INACTIVE_GAS = 1818

  def setUp(self):
    self.packer = CANPackerSafety("subaru_global_2017_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, self.FLAGS)
    self.safety.init_tests()

  def _set_prev_torque(self, t):
    self.safety.set_desired_torque_last(t)
    self.safety.set_rt_torque_last(t)

  def _torque_driver_msg(self, torque):
    values = {"Steer_Torque_Sensor": torque}
    return self.packer.make_can_msg_safety("Steering_Torque", 0, values)

  def _speed_msg(self, speed):
    values = {s: speed for s in ["FR", "FL", "RR", "RL"]}
    return self.packer.make_can_msg_safety("Wheel_Speeds", self.ALT_MAIN_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"Brake": brake}
    return self.packer.make_can_msg_safety("Brake_Status", self.ALT_MAIN_BUS, values)

  def _user_gas_msg(self, gas):
    values = {"Throttle_Pedal": gas}
    return self.packer.make_can_msg_safety("Throttle", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"Cruise_Activated": enable}
    return self.packer.make_can_msg_safety("CruiseControl", self.ALT_MAIN_BUS, values)

  def _comfort_control_msg(self, avh_request, signal2=0x01, signal5=0x0e):
    # the frame the car sends with the engine running. the defaults are the constants the
    # safety pins, so a test that wants a rejected frame overrides one of them
    values = {"AVH_REQUEST": avh_request, "Signal2": signal2, "Signal5": signal5}
    return self.packer.make_can_msg_safety("Comfort_Control", SUBARU_ALT_BUS, values)

  def _stop_start_msg(self, pressed):
    values = {"STOP_START": pressed}
    return self.packer.make_can_msg_safety("Dashlights", SUBARU_ALT_BUS, values)


class TestSubaruStockLongitudinalSafetyBase(TestSubaruSafetyBase):
  def _cancel_msg(self, cancel, cruise_throttle=0):
    values = {"Cruise_Cancel": cancel, "Cruise_Throttle": cruise_throttle}
    return self.packer.make_can_msg_safety("ES_Distance", self.ALT_MAIN_BUS, values)

  def test_cancel_message(self):
    # test that we can only send the cancel message (ES_Distance) with inactive throttle (1818) and Cruise_Cancel=1
    for cancel in [True, False]:
      self._generic_limit_safety_check(partial(self._cancel_msg, cancel), self.INACTIVE_GAS, self.INACTIVE_GAS, 0, 2**12, 1, self.INACTIVE_GAS, cancel)


class TestSubaruLongitudinalSafetyBase(TestSubaruSafetyBase, common.LongitudinalGasBrakeSafetyTest):
  MIN_GAS = 808
  MAX_GAS = 3400
  INACTIVE_GAS = 1818
  MAX_POSSIBLE_GAS = 2**13

  MIN_BRAKE = 0
  MAX_BRAKE = 600
  MAX_POSSIBLE_BRAKE = 2**16

  MIN_RPM = 0
  MAX_RPM = 3600
  MAX_POSSIBLE_RPM = 2**13

  FWD_BLACKLISTED_ADDRS = {2: [SubaruMsg.ES_LKAS, SubaruMsg.ES_Brake, SubaruMsg.ES_Distance,
                               SubaruMsg.ES_Status, SubaruMsg.ES_DashStatus,
                               SubaruMsg.ES_LKAS_State, SubaruMsg.ES_Infotainment]}

  def test_rpm_safety_check(self):
    self._generic_limit_safety_check(self._send_rpm_msg, self.MIN_RPM, self.MAX_RPM, 0, self.MAX_POSSIBLE_RPM, 1)

  def _send_brake_msg(self, brake):
    values = {"Brake_Pressure": brake}
    return self.packer.make_can_msg_safety("ES_Brake", self.ALT_MAIN_BUS, values)

  def _send_gas_msg(self, gas):
    values = {"Cruise_Throttle": gas}
    return self.packer.make_can_msg_safety("ES_Distance", self.ALT_MAIN_BUS, values)

  def _send_rpm_msg(self, rpm):
    values = {"Cruise_RPM": rpm}
    return self.packer.make_can_msg_safety("ES_Status", self.ALT_MAIN_BUS, values)


class TestSubaruTorqueSafetyBase(TestSubaruSafetyBase, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):
  MAX_RATE_UP = 50
  MAX_RATE_DOWN = 70
  MAX_TORQUE_LOOKUP = [0], [2047]

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 7
  MAX_INVALID_STEERING_FRAMES = 1
  STEER_STEP = 2

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_Output": torque, "LKAS_Request": steer_req}
    return self.packer.make_can_msg_safety("ES_LKAS", SUBARU_MAIN_BUS, values)


class TestSubaruGen1TorqueStockLongitudinalSafety(TestSubaruStockLongitudinalSafetyBase, TestSubaruTorqueSafetyBase):
  FLAGS = 0
  TX_MSGS = lkas_tx_msgs(SUBARU_MAIN_BUS)


class TestSubaruGen2TorqueSafetyBase(TestSubaruTorqueSafetyBase):
  ALT_MAIN_BUS = SUBARU_ALT_BUS
  ALT_CAM_BUS = SUBARU_ALT_BUS

  MAX_RATE_UP = 40
  MAX_RATE_DOWN = 40
  MAX_TORQUE_LOOKUP = [0], [1000]


class TestSubaruGen2TorqueStockLongitudinalSafety(TestSubaruStockLongitudinalSafetyBase, TestSubaruGen2TorqueSafetyBase):
  FLAGS = SubaruSafetyFlags.GEN2
  TX_MSGS = lkas_tx_msgs(SUBARU_ALT_BUS)


class TestSubaruGen1LongitudinalSafety(TestSubaruLongitudinalSafetyBase, TestSubaruTorqueSafetyBase):
  FLAGS = SubaruSafetyFlags.LONG
  TX_MSGS = lkas_tx_msgs(SUBARU_MAIN_BUS) + long_tx_msgs(SUBARU_MAIN_BUS)
  RELAY_MALFUNCTION_ADDRS = {SUBARU_MAIN_BUS: (SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                                               SubaruMsg.ES_Infotainment, SubaruMsg.ES_Brake, SubaruMsg.ES_Status,
                                               SubaruMsg.ES_Distance)}


class TestSubaruGen2LongitudinalSafety(TestSubaruLongitudinalSafetyBase, TestSubaruGen2TorqueSafetyBase):
  FLAGS = SubaruSafetyFlags.LONG | SubaruSafetyFlags.GEN2
  TX_MSGS = lkas_tx_msgs(SUBARU_ALT_BUS) + long_tx_msgs(SUBARU_ALT_BUS) + gen2_long_additional_tx_msgs()
  FWD_BLACKLISTED_ADDRS = {2: [SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                               SubaruMsg.ES_Infotainment]}
  RELAY_MALFUNCTION_ADDRS = {SUBARU_MAIN_BUS: (SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                                               SubaruMsg.ES_Infotainment),
                             SUBARU_ALT_BUS: (SubaruMsg.ES_Brake, SubaruMsg.ES_Status, SubaruMsg.ES_Distance)}

  def _rdbi_msg(self, did: int):
    return b'\x03\x22' + did.to_bytes(2) + b'\x00\x00\x00\x00'

  def _es_uds_msg(self, msg: bytes):
    return libsafety_py.make_CANPacket(SubaruMsg.ES_UDS_Request, 2, msg)

  def test_es_uds_message(self):
    tester_present = b'\x02\x3E\x80\x00\x00\x00\x00\x00'
    not_tester_present = b"\x03\xAA\xAA\x00\x00\x00\x00\x00"

    button_did = 0x1130

    # Tester present is allowed for gen2 long to keep eyesight disabled
    self.assertTrue(self._tx(self._es_uds_msg(tester_present)))

    # Non-Tester present is not allowed
    self.assertFalse(self._tx(self._es_uds_msg(not_tester_present)))

    # Only button_did is allowed to be read via UDS
    for did in range(0xFFFF):
      should_tx = (did == button_did)
      self.assertEqual(self._tx(self._es_uds_msg(self._rdbi_msg(did))), should_tx)

    # any other msg is not allowed
    for sid in range(0xFF):
      msg = b'\x03' + sid.to_bytes(1) + b'\x00' * 6
      self.assertFalse(self._tx(self._es_uds_msg(msg)))


class TestSubaruAngleSafetyBase(TestSubaruSafetyBase, common.AngleSteeringSafetyTest):
  STEER_ANGLE_MAX = 190
  DEG_TO_CAN = 100

  # VM-based limits, not breakpoint-based
  ANGLE_RATE_BP = None
  ANGLE_RATE_UP = None
  ANGLE_RATE_DOWN = None

  LATERAL_FREQUENCY = 50

  cnt_angle_cmd = 0

  def setUp(self):
    self.__class__.cnt_angle_cmd = 0
    super().setUp()
    from opendbc.car.subaru.carcontroller import get_safety_CP
    self.VM = VehicleModel(get_safety_CP())

  def _speed_msg(self, speed):
    # speed is in m/s for angle tests, convert to kph for DBC
    speed_kph = speed * 3.6
    values = {s: speed_kph for s in ["FR", "FL", "RR", "RL"]}
    return self.packer.make_can_msg_safety("Wheel_Speeds", self.ALT_MAIN_BUS, values)

  def _angle_cmd_msg(self, angle, enabled, increment_timer=True):
    values = {"LKAS_Output": angle, "LKAS_Request": enabled, "SET_3": 3}
    if increment_timer:
      self.safety.set_timer(self.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    return self.packer.make_can_msg_safety("ES_LKAS_ANGLE", SUBARU_MAIN_BUS, values)

  def _angle_meas_msg(self, angle):
    values = {"Steering_Angle": angle}
    return self.packer.make_can_msg_safety("Steering_2", SUBARU_MAIN_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"Cruise_Activated": enable}
    return self.packer.make_can_msg_safety("ES_Status", self.ALT_MAIN_BUS, values)

  def test_angle_cmd_when_enabled(self):
    # VM-based limits are tested below
    pass

  def _find_max_allowed_angle_can(self, sign):
    """Binary search for the exact max angle CAN value the safety allows."""
    lo, hi = 0, self.STEER_ANGLE_MAX * self.DEG_TO_CAN + 10
    while lo < hi:
      mid = (lo + hi + 1) // 2
      self.safety.set_desired_angle_last(mid * sign)
      if self._tx(self._angle_cmd_msg(mid / self.DEG_TO_CAN * sign, True)):
        lo = mid
      else:
        hi = mid - 1
    return lo

  def test_lateral_accel_limit(self):
    for speed in np.linspace(0, 40, 100):
      speed = max(speed, 1)
      # match Wheel_Speeds rounding on CAN (factor 0.057 kph)
      speed = round_speed(away_round(speed * 3.6 / 0.057) * 0.057 / 3.6)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)
        self._tx(self._angle_cmd_msg(0, True))

        # find the exact max angle CAN the safety allows
        max_angle_can = self._find_max_allowed_angle_can(sign)

        # at limit
        self.safety.set_desired_angle_last(max_angle_can * sign)
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_can / self.DEG_TO_CAN * sign, True)))

        # 1 unit above limit
        above_limit_can = max_angle_can + 1
        self.safety.set_desired_angle_last(above_limit_can * sign)
        self._tx(self._angle_cmd_msg(above_limit_can / self.DEG_TO_CAN * sign, True))

        # at low speeds max angle is above STEER_ANGLE_MAX, so adding 1 has no effect
        should_tx = max_angle_can >= self.STEER_ANGLE_MAX * self.DEG_TO_CAN
        self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(above_limit_can / self.DEG_TO_CAN * sign, True)))

  def _find_max_allowed_delta_can(self, sign):
    """Binary search for the exact max angle delta CAN value the safety allows from angle 0."""
    lo, hi = 0, self.STEER_ANGLE_MAX * self.DEG_TO_CAN
    while lo < hi:
      mid = (lo + hi + 1) // 2
      # reset to angle 0
      self.safety.set_desired_angle_last(0)
      if self._tx(self._angle_cmd_msg(mid / self.DEG_TO_CAN * sign, True)):
        lo = mid
      else:
        hi = mid - 1
    return lo

  def test_lateral_jerk_limit(self):
    for speed in np.linspace(0, 40, 100):
      speed = max(speed, 1)
      # match Wheel_Speeds rounding on CAN
      speed = round_speed(away_round(speed * 3.6 / 0.057) * 0.057 / 3.6)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)
        self._tx(self._angle_cmd_msg(0, True))

        # find the exact max delta CAN the safety allows from angle 0
        max_delta_can = self._find_max_allowed_delta_can(sign)

        # Stay within limits
        # Up
        self.safety.set_desired_angle_last(0)
        self.assertTrue(self._tx(self._angle_cmd_msg(max_delta_can / self.DEG_TO_CAN * sign, True)))

        # Don't change
        self.assertTrue(self._tx(self._angle_cmd_msg(max_delta_can / self.DEG_TO_CAN * sign, True)))

        # Down
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

        # Inject too high rates
        # Up
        above_delta_can = max_delta_can + 1
        self.assertFalse(self._tx(self._angle_cmd_msg(above_delta_can / self.DEG_TO_CAN * sign, True)))

        # Don't change
        self.safety.set_desired_angle_last(round(above_delta_can * sign))
        self.assertTrue(self._tx(self._angle_cmd_msg(above_delta_can / self.DEG_TO_CAN * sign, True)))

        # Down
        self.assertFalse(self._tx(self._angle_cmd_msg(0, True)))

        # Recover
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))


class TestSubaruGen1AngleStockLongitudinalSafety(TestSubaruStockLongitudinalSafetyBase, TestSubaruAngleSafetyBase):
  FLAGS = SubaruSafetyFlags.LKAS_ANGLE
  TX_MSGS = lkas_tx_msgs(SUBARU_MAIN_BUS, SubaruMsg.ES_LKAS_ANGLE, lkas_alert=True)
  RELAY_MALFUNCTION_ADDRS = ANGLE_RELAY_MALFUNCTION_ADDRS
  FWD_BLACKLISTED_ADDRS = fwd_blacklisted_addr(SubaruMsg.ES_LKAS_ANGLE, lkas_alert=True)


class TestSubaruGen2AngleStockLongitudinalSafety(TestSubaruStockLongitudinalSafetyBase, TestSubaruAngleSafetyBase):
  ALT_MAIN_BUS = SUBARU_ALT_BUS
  FLAGS = SubaruSafetyFlags.GEN2 | SubaruSafetyFlags.LKAS_ANGLE
  TX_MSGS = lkas_tx_msgs(SUBARU_ALT_BUS, SubaruMsg.ES_LKAS_ANGLE, lkas_alert=True) + comfort_tx_msgs(SUBARU_ALT_BUS)
  RELAY_MALFUNCTION_ADDRS = ANGLE_RELAY_MALFUNCTION_ADDRS
  FWD_BLACKLISTED_ADDRS = fwd_blacklisted_addr(SubaruMsg.ES_LKAS_ANGLE, lkas_alert=True)

  # whether openpilot was configured to ask for the comfort settings. the subclass below
  # turns them on; here they are off, which is the default every car ships with.
  COMFORT = False

  def test_comfort_avh_flag(self):
    # the AVH request is refused outright unless openpilot was told to ask for it
    self.safety.set_controls_allowed(True)
    self.assertEqual(self.COMFORT, self._tx(self._comfort_control_msg(2)))
    self.assertEqual(self.COMFORT, self._tx(self._comfort_control_msg(1)))

  def test_comfort_stop_start_flag(self):
    self.safety.set_controls_allowed(True)
    self.assertEqual(self.COMFORT, self._tx(self._stop_start_msg(True)))

  def test_avh_refused_while_moving(self):
    # AVH is the one request that touches the brakes, so it can never reach a rolling car
    self.safety.set_controls_allowed(True)
    self._rx(self._speed_msg(10))
    self.assertTrue(self.safety.get_vehicle_moving())
    self.assertFalse(self._tx(self._comfort_control_msg(2)))

  def test_stop_start_allowed_while_moving(self):
    # the start-stop button touches nothing but the engine's own idle stop, and openpilot is
    # often still starting up as the driver pulls away, so this one is not speed gated
    self.safety.set_controls_allowed(True)
    self._rx(self._speed_msg(10))
    self.assertTrue(self.safety.get_vehicle_moving())
    self.assertEqual(self.COMFORT, self._tx(self._stop_start_msg(True)))

  def test_comfort_control_pinned_content(self):
    # openpilot may send this one frame and no other: an out of range request, or any change
    # to the bytes the car holds constant, is a violation even with the flag on
    self.safety.set_controls_allowed(True)
    for bad_request in (3, 0x80, 0xff):
      self.assertFalse(self._tx(self._comfort_control_msg(bad_request)), f"{bad_request=}")
    self.assertFalse(self._tx(self._comfort_control_msg(2, signal2=0x00)))
    self.assertFalse(self._tx(self._comfort_control_msg(2, signal5=0x00)))

  def test_stop_start_press_only(self):
    # a frame with the button bit clear is openpilot competing with the car's own copy of a
    # message it has no business sending, so it is refused however the flags are set
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._stop_start_msg(False)))


class TestSubaruGen2AngleMadsSafety(TestSubaruStockLongitudinalSafetyBase, TestSubaruAngleSafetyBase):
  """MADS: lateral latches on the first ACC engage and only exits on the main switch.

  Everything the non-MADS angle car does still applies, so this inherits the full angle
  suite. Only the ACC-dropout behavior differs, and that override is below.
  """
  ALT_MAIN_BUS = SUBARU_ALT_BUS
  FLAGS = SubaruSafetyFlags.GEN2 | SubaruSafetyFlags.LKAS_ANGLE | SubaruSafetyFlags.MADS
  # whether the main switch arms on its own, so the shared tests below cover both variants
  MADS_MAIN = False
  TX_MSGS = lkas_tx_msgs(SUBARU_ALT_BUS, SubaruMsg.ES_LKAS_ANGLE, lkas_alert=True) + comfort_tx_msgs(SUBARU_ALT_BUS)
  RELAY_MALFUNCTION_ADDRS = ANGLE_RELAY_MALFUNCTION_ADDRS
  FWD_BLACKLISTED_ADDRS = fwd_blacklisted_addr(SubaruMsg.ES_LKAS_ANGLE, lkas_alert=True)

  def setUp(self):
    super().setUp()
    # EyeSight main defaults on with the car running, so this is the resting state
    self._rx(self._main_switch_msg(True))

  def _main_switch_msg(self, on):
    # the camera's own copy on the camera bus, not the one we transmit on the main bus
    values = {"Cruise_On": on}
    return self.packer.make_can_msg_safety("ES_DashStatus", SUBARU_CAM_BUS, values)

  def _arm(self):
    self._rx(self._main_switch_msg(True))
    self._rx(self._pcm_status_msg(False))
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def _assert_brake_holds_lateral_only(self):
    for _ in range(3):
      self._rx(self._user_brake_msg(1))
      # the brake takes longitudinal authority away exactly as it does on a stock car
      self.assertFalse(self.safety.get_controls_allowed())
      # and lane centering carries on regardless, which is the point of MADS
      self.assertTrue(self.safety.get_controls_allowed_lateral())
      self._rx(self._user_brake_msg(0))
      # released, longitudinal authority comes back
      self.assertTrue(self.safety.get_controls_allowed())
      self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_allow_user_brake_at_zero_speed(self):
    # OVERRIDE: the base asserts a brake rising edge exits controls entirely. under MADS it
    # exits longitudinal only, and controls_allowed_lateral carries the steering.
    self._arm()
    self._rx(self._vehicle_moving_msg(0))
    self._assert_brake_holds_lateral_only()

  def test_not_allow_user_brake_when_moving(self):
    # OVERRIDE: same, and at speed, which is the case that faulted the EPS on the road
    self._arm()
    self._rx(self._vehicle_moving_msg(self.STANDSTILL_THRESHOLD + 1))
    self._assert_brake_holds_lateral_only()

  def test_brake_blocks_longitudinal_while_lateral_holds(self):
    # the reason controls_allowed_lateral exists. ES_Distance is the only longitudinal
    # message this car can transmit, and a held brake must refuse it even though steering
    # is still live. an inactive-throttle cancel stays allowed, that is a decel request.
    self._arm()
    self._rx(self._vehicle_moving_msg(self.STANDSTILL_THRESHOLD + 1))
    self._rx(self._user_brake_msg(1))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self.safety.get_longitudinal_allowed())

  def test_steering_still_tx_while_braking(self):
    # the other half: the steering command itself must still be accepted through the brake
    self._arm()
    self._rx(self._vehicle_moving_msg(self.STANDSTILL_THRESHOLD + 1))
    self._rx(self._user_brake_msg(1))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

  def test_brake_release_does_not_rearm_a_disarmed_car(self):
    # the re-sync is gated on controls_allowed_lateral so a brake release can never hand
    # controls back to a car that was not holding lateral in the first place
    self._rx(self._main_switch_msg(True))
    self._rx(self._vehicle_moving_msg(self.STANDSTILL_THRESHOLD + 1))
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self._rx(self._user_brake_msg(1))
    self._rx(self._user_brake_msg(0))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_rx_lag_exits_lateral(self):
    # and so must a stale rx check, which is what safety_tick enforces
    self._arm()
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.safety.set_timer(10_000_000)
    self.safety.safety_tick_current_safety_config()
    self.assertFalse(self.safety.safety_config_valid())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_brake_still_exits_controls_without_mads(self):
    # the gate in generic_rx_checks is global, so prove it is genuinely per mode and that
    # a plain angle car is unaffected
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru,
                                 SubaruSafetyFlags.GEN2 | SubaruSafetyFlags.LKAS_ANGLE)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    self._rx(self._vehicle_moving_msg(self.STANDSTILL_THRESHOLD + 1))
    self._rx(self._user_brake_msg(1))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_mads_does_not_leak_into_another_safety_mode(self):
    # controls_allowed_lateral is global and only subaru ever sets it, so a missing reset
    # in set_safety_hooks would leave another brand steering on a flag it never granted.
    self._arm()
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.safety.set_safety_hooks(CarParams.SafetyModel.toyota, 0)
    self.safety.init_tests()
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_mads_refused_with_openpilot_longitudinal(self):
    # a scope limit rather than a safety necessity now, but still refused
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, self.FLAGS | SubaruSafetyFlags.LONG)
    self.safety.init_tests()
    self._rx(self._main_switch_msg(True))
    self._rx(self._pcm_status_msg(False))
    self._rx(self._pcm_status_msg(True))
    self._rx(self._vehicle_moving_msg(self.STANDSTILL_THRESHOLD + 1))
    self._rx(self._user_brake_msg(1))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_disable_control_allowed_from_cruise(self):
    # overrides the stock pcm behavior. ACC dropping out must NOT exit controls, that is
    # the entire point of MADS
    self._arm()
    self._rx(self._pcm_status_msg(False))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_not_armed_before_first_engage(self):
    # main on by itself steers nothing until the driver engages ACC once
    for _ in range(100):
      self._rx(self._main_switch_msg(True))
      self._rx(self._pcm_status_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_latches_through_sustained_acc_dropout(self):
    self._arm()
    for _ in range(1000):
      self._rx(self._pcm_status_msg(False))
      self._rx(self._main_switch_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_main_switch_off_exits_controls(self):
    self._arm()
    self._rx(self._main_switch_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_main_switch_off_wins_over_engaged_acc(self):
    # ACC held high while the main switch goes off must not keep or regain controls
    self._arm()
    self._rx(self._main_switch_msg(False))
    for _ in range(100):
      self._rx(self._pcm_status_msg(True))
      self._rx(self._main_switch_msg(False))
      self.assertFalse(self.safety.get_controls_allowed())

  def test_no_rearm_without_a_fresh_acc_edge(self):
    # after a main switch cycle, ACC still held high is not a rising edge
    self._arm()
    self._rx(self._main_switch_msg(False))
    self._rx(self._main_switch_msg(True))
    for _ in range(100):
      self._rx(self._pcm_status_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_rearm_after_main_switch_cycle(self):
    self._arm()
    self._rx(self._main_switch_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._main_switch_msg(True))
    self._rx(self._pcm_status_msg(False))
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_no_arm_while_main_switch_off(self):
    # a full ACC engage cycle with the main switch off must never allow controls
    self._rx(self._main_switch_msg(False))
    for _ in range(100):
      self._rx(self._pcm_status_msg(False))
      self._rx(self._pcm_status_msg(True))
      self.assertFalse(self.safety.get_controls_allowed())

  def test_acc_main_on_tracks_the_signal(self):
    for on in (True, False, True, False):
      self._rx(self._main_switch_msg(on))
      self.assertEqual(on, self.safety.get_acc_main_on())

  def test_matches_the_openpilot_side_latch(self):
    # carstate has to agree with the panda about lateral or the heartbeat check clears
    # controls after 3s. worse, openpilot keeps commanding an angle the panda refuses to
    # send, the EPS sees the LKAS stream stop mid-engagement and latches a steer fault.
    #
    # MadsLatch is openpilot's lateral latch, so controls_allowed_lateral is what it has to
    # match. brake is in the sequence precisely because it is not an input to MadsLatch:
    # this is what proves the lateral flag ignores it too. an earlier version compared
    # against controls_allowed, which only worked while mads_enabled suppressed the brake.
    #
    # controls_allowed is pinned in the same loop, since the brake dropping it and the
    # release restoring it is the whole behavior this change introduces.
    from opendbc.car.subaru.carstate import MadsLatch

    inputs = list(itertools.product((False, True), repeat=3))
    for seq in itertools.product(inputs, repeat=3):
      with self.subTest(seq=seq):
        self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, self.FLAGS)
        self.safety.init_tests()
        self._rx(self._vehicle_moving_msg(self.STANDSTILL_THRESHOLD + 1))
        latch = MadsLatch()

        for main_on, acc_enabled, brake in seq:
          self._rx(self._main_switch_msg(main_on))
          self._rx(self._pcm_status_msg(acc_enabled))
          self._rx(self._user_brake_msg(brake))
          lateral = latch.update(main_on, acc_enabled, self.MADS_MAIN)
          self.assertEqual(lateral, self.safety.get_controls_allowed_lateral())
          self.assertEqual(lateral and not brake, self.safety.get_controls_allowed())


class TestSubaruGen2AngleMadsMainSafety(TestSubaruGen2AngleMadsSafety):
  """MADS with the main switch arming on its own, so no set speed is needed.

  Everything the plain MADS car does still applies, including arming off ACC, so this
  inherits the whole suite. Only the extra entry path differs.
  """
  FLAGS = SubaruSafetyFlags.GEN2 | SubaruSafetyFlags.LKAS_ANGLE | SubaruSafetyFlags.MADS | SubaruSafetyFlags.MADS_MAIN
  MADS_MAIN = True

  def _arm(self):
    # the whole point: a main switch tap, no ACC anywhere in the sequence
    self._rx(self._main_switch_msg(False))
    self._rx(self._main_switch_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_no_rearm_without_a_fresh_acc_edge(self):
    # OVERRIDE: the base proves a main switch cycle alone cannot re-arm. here it is
    # supposed to, so assert the opposite and keep the coverage rather than dropping it
    self._arm()
    self._rx(self._main_switch_msg(False))
    self._rx(self._main_switch_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_the_switch_already_being_on_is_not_an_edge(self):
    # the car starts with EyeSight main on, so the first message is not a driver action.
    # arming there would land while openpilot is still initializing, and the panda would
    # drop controls 3s later on heartbeat mismatch
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, self.FLAGS)
    self.safety.init_tests()
    for _ in range(100):
      self._rx(self._main_switch_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_main_switch_tap_arms_without_acc(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, self.FLAGS)
    self.safety.init_tests()
    self._rx(self._main_switch_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._main_switch_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_acc_engage_still_arms(self):
    # the ACC path is additive, not replaced
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, self.FLAGS)
    self.safety.init_tests()
    self._rx(self._main_switch_msg(True))
    self._rx(self._pcm_status_msg(False))
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_main_switch_arming_needs_mads(self):
    # the flag on its own must not turn a plain angle car into a main switch engage
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru,
                                 SubaruSafetyFlags.GEN2 | SubaruSafetyFlags.LKAS_ANGLE | SubaruSafetyFlags.MADS_MAIN)
    self.safety.init_tests()
    self._rx(self._main_switch_msg(False))
    self._rx(self._main_switch_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_main_switch_arming_refused_with_openpilot_longitudinal(self):
    # MADS is refused there, and this rides on MADS, so it has to go too
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, self.FLAGS | SubaruSafetyFlags.LONG)
    self.safety.init_tests()
    self._rx(self._main_switch_msg(False))
    self._rx(self._main_switch_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())


class TestSubaruGen2AngleComfortSafety(TestSubaruGen2AngleStockLongitudinalSafety):
  """The same angle car with openpilot allowed to ask for the two comfort settings.

  Everything the plain angle car does still applies, so this inherits the whole suite. The
  gates on content and on the car moving hold either way; only whether a well formed request
  is allowed at all changes, which is what COMFORT flips.
  """
  FLAGS = SubaruSafetyFlags.GEN2 | SubaruSafetyFlags.LKAS_ANGLE | SubaruSafetyFlags.AVH | SubaruSafetyFlags.STOP_START
  COMFORT = True

  def test_comfort_needs_a_gen2_angle_car(self):
    # the messages were only ever measured on the alt bus of a gen2 angle car, so the flags
    # do nothing anywhere else however openpilot sets them
    for flags in (SubaruSafetyFlags.GEN2, SubaruSafetyFlags.LKAS_ANGLE):
      self.safety.set_safety_hooks(CarParams.SafetyModel.subaru,
                                   flags | SubaruSafetyFlags.AVH | SubaruSafetyFlags.STOP_START)
      self.safety.init_tests()
      self.safety.set_controls_allowed(True)
      self.assertFalse(self._tx(self._comfort_control_msg(2)), f"{flags=}")
      self.assertFalse(self._tx(self._stop_start_msg(True)), f"{flags=}")

  def test_each_comfort_flag_only_opens_its_own_message(self):
    # AVH is a brake function and start-stop is engine only, so they are asked for
    # separately and must not let each other through
    for flag, avh_ok, stop_start_ok in ((SubaruSafetyFlags.AVH, True, False),
                                        (SubaruSafetyFlags.STOP_START, False, True)):
      self.safety.set_safety_hooks(CarParams.SafetyModel.subaru,
                                   SubaruSafetyFlags.GEN2 | SubaruSafetyFlags.LKAS_ANGLE | flag)
      self.safety.init_tests()
      self.safety.set_controls_allowed(True)
      self.assertEqual(avh_ok, self._tx(self._comfort_control_msg(2)), f"{flag=}")
      self.assertEqual(stop_start_ok, self._tx(self._stop_start_msg(True)), f"{flag=}")


if __name__ == "__main__":
  unittest.main()
