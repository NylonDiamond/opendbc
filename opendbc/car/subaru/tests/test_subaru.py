import unittest
from types import SimpleNamespace

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus
from opendbc.car.car_helpers import interfaces
from opendbc.car.structs import CarControl
from opendbc.car.subaru import subarucan
from opendbc.car.subaru.carcontroller import CarController, RESUME_MAX_ATTEMPTS, RESUME_MIN_HOLD_FRAMES, RESUME_PULSE_SENDS
from opendbc.car.subaru.fingerprints import FW_VERSIONS
from opendbc.car.subaru.carstate import MadsLatch
from opendbc.car.subaru.values import CAR, DBC, SubaruSafetyFlags, enable_auto_resume, enable_mads, enable_mads_main, \
                                      is_auto_resume_enabled, is_mads_enabled, is_mads_main_enabled

VisualAlert = CarControl.HUDControl.VisualAlert


class TestSubaruMads(unittest.TestCase):
  """openpilot turns MADS on after the interface is built, so nothing may cache it at
  construction. A stale copy leaves CarState disagreeing with the panda about whether
  controls are allowed, which shows up on the road as steering dropping out 3s later.
  """
  PLATFORM = CAR.SUBARU_CROSSTREK_2024

  def _get_params(self):
    return interfaces[self.PLATFORM].get_params(self.PLATFORM, {0: {}, 1: {}, 2: {}}, [], False, False, docs=False)

  def test_safety_param_survives_capnp(self):
    # capnp rejects an IntFlag, so a bare |= here crashes card before it publishes CarParams
    CP = self._get_params()
    enable_mads(CP)
    self.assertTrue(CP.safetyConfigs[0].safetyParam & SubaruSafetyFlags.MADS)
    # the round trip is how every other process receives it
    self.assertTrue(is_mads_enabled(CP.as_reader()))

  def test_leaves_the_other_flags_alone(self):
    CP = self._get_params()
    before = CP.safetyConfigs[0].safetyParam
    enable_mads(CP)
    self.assertEqual(CP.safetyConfigs[0].safetyParam, before | SubaruSafetyFlags.MADS)

  def test_carstate_sees_mads_enabled_after_init(self):
    CP = self._get_params()
    CI = interfaces[self.PLATFORM](CP)
    self.assertFalse(CI.CS.mads_enabled)
    enable_mads(CP)
    self.assertTrue(CI.CS.mads_enabled)

  def test_carstate_stays_off_without_the_flag(self):
    CP = self._get_params()
    CI = interfaces[self.PLATFORM](CP)
    self.assertFalse(CI.CS.mads_enabled)


class TestSubaruMadsMain(TestSubaruMads):
  """Arming off the cruise main switch is set the same way and read back the same way."""

  def test_safety_param_survives_capnp(self):
    CP = self._get_params()
    enable_mads_main(CP)
    self.assertTrue(CP.safetyConfigs[0].safetyParam & SubaruSafetyFlags.MADS_MAIN)
    self.assertTrue(is_mads_main_enabled(CP.as_reader()))

  def test_leaves_the_other_flags_alone(self):
    CP = self._get_params()
    enable_mads(CP)
    before = CP.safetyConfigs[0].safetyParam
    enable_mads_main(CP)
    self.assertEqual(CP.safetyConfigs[0].safetyParam, before | SubaruSafetyFlags.MADS_MAIN)
    # both paths still read true, since one rides on the other
    self.assertTrue(is_mads_enabled(CP))
    self.assertTrue(is_mads_main_enabled(CP))

  def test_carstate_sees_mads_enabled_after_init(self):
    CP = self._get_params()
    CI = interfaces[self.PLATFORM](CP)
    self.assertFalse(CI.CS.mads_main_enabled)
    enable_mads_main(CP)
    self.assertTrue(CI.CS.mads_main_enabled)

  def test_carstate_stays_off_without_the_flag(self):
    CP = self._get_params()
    CI = interfaces[self.PLATFORM](CP)
    self.assertFalse(CI.CS.mads_main_enabled)
    # MADS on its own must not imply it
    enable_mads(CP)
    self.assertFalse(CI.CS.mads_main_enabled)


class TestMadsLatch(unittest.TestCase):
  """The arming rules, stated directly. test_subaru.py in the safety tests drives this
  against the C implementation; this is what the behavior is supposed to be.
  """

  def test_acc_edge_arms(self):
    latch = MadsLatch()
    self.assertFalse(latch.update(True, False, False))
    self.assertTrue(latch.update(True, True, False))

  def test_main_switch_alone_does_nothing_without_the_flag(self):
    latch = MadsLatch()
    self.assertFalse(latch.update(False, False, False))
    self.assertFalse(latch.update(True, False, False))

  def test_main_switch_tap_arms_with_the_flag(self):
    latch = MadsLatch()
    self.assertFalse(latch.update(False, False, True))
    self.assertTrue(latch.update(True, False, True))

  def test_the_switch_already_being_on_is_not_an_edge(self):
    # the car starts with EyeSight main on, and arming there lands while openpilot is
    # still initializing
    latch = MadsLatch()
    for _ in range(100):
      self.assertFalse(latch.update(True, False, True))

  def test_main_switch_off_still_exits(self):
    latch = MadsLatch()
    latch.update(False, False, True)
    self.assertTrue(latch.update(True, False, True))
    self.assertFalse(latch.update(False, False, True))

  def test_holds_through_an_acc_dropout(self):
    latch = MadsLatch()
    latch.update(False, False, True)
    self.assertTrue(latch.update(True, False, True))
    for _ in range(100):
      self.assertTrue(latch.update(True, True, True))
      self.assertTrue(latch.update(True, False, True))

  def test_turning_the_flag_off_does_not_disarm(self):
    # the toggle only applies at car init, so it cannot change mid drive. prove the latch
    # does not treat it as an exit anyway
    latch = MadsLatch()
    latch.update(False, False, True)
    self.assertTrue(latch.update(True, False, True))
    self.assertTrue(latch.update(True, False, False))


class TestSubaruFingerprint(unittest.TestCase):
  def test_fw_version_format(self):
    for platform, fws_per_ecu in FW_VERSIONS.items():
      for (ecu, _, _), fws in fws_per_ecu.items():
        fw_size = len(fws[0])
        for fw in fws:
          assert len(fw) == fw_size, f"{platform} {ecu}: {len(fw)} {fw_size}"


class TestSubaruLkasAlert(unittest.TestCase):
  """The camera repeats its LKAS alert on ES_LKAS_Alert, which carries nothing else.

  ES_LKAS_State is filtered but the dash reads this copy too, so the stock "Keep hands on
  wheel" nag survives unless both are cleared. Byte values here are taken from a 2024
  Crosstrek: LKAS_Alert_State reads 4 for as long as a message is up, so it has to go with it.
  """
  DBC = "subaru_global_2017_generated"

  def setUp(self):
    self.packer = CANPacker(self.DBC)
    self.parser = CANParser(self.DBC, [("ES_LKAS_Alert", 0)], 0)

  def _build(self, alert_msg, alert=0, state=0, signal2=0, visual_alert=VisualAlert.none):
    camera = {"CHECKSUM": 0, "COUNTER": 0, "Signal1": 0, "LKAS_Alert": alert,
              "LKAS_Alert_Msg": alert_msg, "LKAS_Alert_State": state,
              "Signal2": signal2, "Signal3": 0}
    msg = subarucan.create_es_lkas_alert(self.packer, 0, camera, visual_alert)
    self.parser.update([0, [msg]])
    return self.parser.vl["ES_LKAS_Alert"]

  def test_hands_on_wheel_is_cleared(self):
    for alert_msg in (1, 7):
      with self.subTest(alert_msg=alert_msg):
        out = self._build(alert_msg, state=4)
        self.assertEqual(out["LKAS_Alert_Msg"], 0)
        self.assertEqual(out["LKAS_Alert_State"], 0)

  def test_audible_nag_is_cleared(self):
    for alert in (27, 28, 30):
      with self.subTest(alert=alert):
        self.assertEqual(self._build(7, alert=alert, state=4)["LKAS_Alert"], 0)

  def test_unrelated_alerts_pass_through(self):
    # 25 is Audio_Lead_Car_Change, nothing to do with hands on wheel
    out = self._build(0, alert=25, state=2, signal2=1)
    self.assertEqual(out["LKAS_Alert"], 25)
    self.assertEqual(out["LKAS_Alert_State"], 2)
    self.assertEqual(out["Signal2"], 1)

  def test_pre_collision_braking_is_not_filtered(self):
    # 6 is Pre_Collision_Braking, which the driver needs to see
    out = self._build(6, state=4)
    self.assertEqual(out["LKAS_Alert_Msg"], 6)
    self.assertEqual(out["LKAS_Alert_State"], 4)

  def test_openpilot_can_raise_its_own_hands_on_wheel(self):
    out = self._build(0, visual_alert=VisualAlert.steerRequired)
    self.assertEqual(out["LKAS_Alert_Msg"], 1)
    self.assertEqual(out["LKAS_Alert_State"], 4)

  def test_counter_advances_and_checksum_is_valid(self):
    camera = {"CHECKSUM": 0, "COUNTER": 0, "Signal1": 0, "LKAS_Alert": 0,
              "LKAS_Alert_Msg": 0, "LKAS_Alert_State": 0, "Signal2": 0, "Signal3": 0}
    for frame in range(20):
      addr, dat, _ = subarucan.create_es_lkas_alert(self.packer, frame, camera, VisualAlert.none)
      dat = bytes(dat)
      self.assertEqual(dat[1] & 0xF, frame % 0x10)
      self.assertEqual(dat[0], (addr % 256 + addr // 256 + sum(dat[1:])) & 0xFF)


class TestSubaruAutoResume(unittest.TestCase):
  """Auto resume out of EyeSight's HOLD state.

  The trigger has to be tight in both directions. Too eager and the car pulls into an
  intersection on its own; too shy and the driver keeps tapping the gas anyway. Numbers here
  come off a real resume: the gap sat in a +/- 0.2 m noise band while stopped, and the lead
  gained 0.5 m about 0.7 s before the driver reached for the button.
  """
  PLATFORM = CAR.SUBARU_CROSSTREK_2024

  def setUp(self):
    self.CP = interfaces[self.PLATFORM].get_params(self.PLATFORM, {0: {}, 1: {}, 2: {}}, [], False, False, docs=False)
    enable_auto_resume(self.CP)
    self.CC = CarController(None, self.CP)

  @staticmethod
  def _state(close_distance, car_follow=True, hold=True, standstill=True, acc=True,
             brake=False, gas=False, enabled=True):
    out = SimpleNamespace(standstill=standstill, stockCruiseEngaged=acc, brakePressed=brake,
                          gasPressed=gas, cruiseState=SimpleNamespace(standstill=hold))
    CS = SimpleNamespace(out=out, es_distance_msg={"Close_Distance": close_distance, "Car_Follow": car_follow})
    return SimpleNamespace(enabled=enabled), CS

  def _run(self, frames, close_distance, **kwargs):
    """Step the controller and return how many resume messages it asked for."""
    sends = 0
    for _ in range(frames):
      dist = close_distance(self.CC.frame) if callable(close_distance) else close_distance
      CC, CS = self._state(dist, **kwargs)
      sends += self.CC._update_auto_resume(CC, CS)
      self.CC.frame += 1
    return sends

  def test_resumes_once_the_lead_pulls_away(self):
    self._run(100, 1.0)                               # settle in HOLD behind a lead
    self.assertEqual(self._run(100, 1.8), RESUME_PULSE_SENDS)

  def test_pulse_is_three_sends(self):
    # 3 sends at 20 Hz is the 150 ms a real button press measured on the car. the literal is
    # the point of the test: written against the constant it would prove nothing about length
    self._run(100, 1.0)
    self.assertEqual(self._run(100, 1.8), 3)

  def test_noise_alone_never_fires(self):
    # the real trace wandered between 0.96 and 1.16 m for 14 s without the lead moving, and it
    # wandered slowly, so the debounce alone does not save us. the threshold has to do the work
    self.assertEqual(self._run(1500, lambda f: 1.06 + 0.1 * (-1) ** (f // 40)), 0)

  def test_no_lead_means_no_resume(self):
    # first in line at a red light. nothing on this car can tell us the light changed
    self.assertEqual(self._run(500, 5.0, car_follow=False), 0)

  def test_does_not_fire_before_settling_into_hold(self):
    # gap opening from the very first frame of HOLD. that is a car still rolling to a stop,
    # not a lead pulling away, so nothing may fire until the settle time is up. the frame
    # count is a literal on purpose: derived from the constant, this test goes vacuous the
    # moment the constant is what breaks
    self.assertLess(45, RESUME_MIN_HOLD_FRAMES + 1)
    self.assertEqual(self._run(45, lambda f: 1.0 + 0.05 * f), 0)

  def test_a_brief_spike_does_not_fire(self):
    # one bad reading is not the lead moving. the gap has to stay open to count
    self.assertEqual(self._run(400, lambda f: 2.0 if 200 <= f < 206 else 1.0), 0)

  def test_driver_on_the_brake_blocks_it(self):
    self._run(100, 1.0)
    self.assertEqual(self._run(100, 1.8, brake=True), 0)

  def test_driver_on_the_gas_blocks_it(self):
    self._run(100, 1.0)
    self.assertEqual(self._run(100, 1.8, gas=True), 0)

  def test_openpilot_disengaged_blocks_it(self):
    self._run(100, 1.0)
    self.assertEqual(self._run(100, 1.8, enabled=False), 0)

  def test_leaving_hold_forgets_the_stop(self):
    self._run(100, 1.0)
    self._run(10, 1.0, hold=False)                    # rolled away
    # back at a stop, but the lead reference is gone with it, so the old gap cannot fire
    self.assertEqual(self._run(100, 1.8), 0)

  def test_retries_a_few_times_if_the_car_ignores_it(self):
    self._run(100, 1.0)
    # gap stays open and the car stays in HOLD, so the press clearly did not take
    total = self._run(3000, 1.8)
    self.assertEqual(total, RESUME_PULSE_SENDS * RESUME_MAX_ATTEMPTS)

  def test_disabled_without_the_flag(self):
    CP = interfaces[self.PLATFORM].get_params(self.PLATFORM, {0: {}, 1: {}, 2: {}}, [], False, False, docs=False)
    self.CC = CarController(None, CP)
    self._run(100, 1.0)
    self.assertEqual(self._run(100, 1.8), 0)

  def test_flag_is_read_live_not_snapshotted(self):
    # enable_auto_resume runs after the interface is built, so a cached copy would be stale
    CP = interfaces[self.PLATFORM].get_params(self.PLATFORM, {0: {}, 1: {}, 2: {}}, [], False, False, docs=False)
    CC = CarController(None, CP)
    self.assertFalse(CC.auto_resume_available)
    enable_auto_resume(CP)
    self.assertTrue(CC.auto_resume_available)

  def test_safety_param_survives_capnp(self):
    CP = interfaces[self.PLATFORM].get_params(self.PLATFORM, {0: {}, 1: {}, 2: {}}, [], False, False, docs=False)
    before = CP.safetyConfigs[0].safetyParam
    enable_auto_resume(CP)
    self.assertEqual(CP.safetyConfigs[0].safetyParam, before | SubaruSafetyFlags.AUTO_RESUME)
    self.assertTrue(is_auto_resume_enabled(CP.as_reader()))


class TestSubaruResumeMessage(unittest.TestCase):
  PLATFORM = CAR.SUBARU_CROSSTREK_2024

  def setUp(self):
    self.packer = CANPacker(DBC[self.PLATFORM][Bus.pt])
    self.parser = CANParser(DBC[self.PLATFORM][Bus.pt], [("ES_Distance", 20)], 1)
    self.camera = {s: 0 for s in ["CHECKSUM", "COUNTER", "Signal1", "Cruise_Fault", "Cruise_Throttle", "Signal2",
                                  "Car_Follow", "Low_Speed_Follow", "Cruise_Soft_Disable", "Signal7",
                                  "Cruise_Brake_Active", "Distance_Swap", "Cruise_EPB", "Signal4", "Close_Distance",
                                  "Signal5", "Cruise_Cancel", "Cruise_Set", "Cruise_Resume", "Signal6"]}

  def _build(self, **kwargs):
    addr, dat, _ = subarucan.create_es_distance(self.packer, 1, self.camera, 1, kwargs.pop("pcm_cancel_cmd", False), **kwargs)
    self.parser.update([(0, [(addr, bytes(dat), 1)])])
    return self.parser.vl["ES_Distance"]

  def test_resume_sets_the_bit_with_inactive_throttle(self):
    out = self._build(cruise_resume_cmd=True)
    self.assertEqual(out["Cruise_Resume"], 1)
    self.assertEqual(out["Cruise_Cancel"], 0)
    self.assertEqual(out["Cruise_Set"], 0)
    self.assertEqual(out["Cruise_Throttle"], 1818)

  def test_cancel_wins_over_resume(self):
    out = self._build(pcm_cancel_cmd=True, cruise_resume_cmd=True)
    self.assertEqual(out["Cruise_Cancel"], 1)
    self.assertEqual(out["Cruise_Resume"], 0)

  def test_neither_bit_set_by_default(self):
    out = self._build()
    self.assertEqual(out["Cruise_Resume"], 0)
    self.assertEqual(out["Cruise_Cancel"], 0)
