import unittest

from opendbc.car.car_helpers import interfaces
from opendbc.car.subaru.fingerprints import FW_VERSIONS
from opendbc.car.subaru.carstate import MadsLatch
from opendbc.car.subaru.values import CAR, SubaruSafetyFlags, enable_mads, enable_mads_main, is_mads_enabled, is_mads_main_enabled


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
