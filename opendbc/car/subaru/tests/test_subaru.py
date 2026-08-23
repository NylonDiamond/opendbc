import unittest
from types import SimpleNamespace

from opendbc.can import CANPacker, CANParser
from opendbc.car.car_helpers import interfaces
from opendbc.car.structs import CarControl
from opendbc.car.subaru import subarucan
from opendbc.car.subaru.carcontroller import CarController, COMFORT_BURST_LEN, COMFORT_DEADLINE_FRAMES, \
                                             COMFORT_PRESS_FRAMES, COMFORT_PRESS_TRIES, COMFORT_SETTLE_FRAMES
from opendbc.car.subaru.fingerprints import FW_VERSIONS
from opendbc.car.subaru.carstate import MadsLatch
from opendbc.car.subaru.values import CAR, CanBus, DBC, SubaruSafetyFlags, enable_avh, enable_mads, enable_mads_main, \
                                      enable_stop_start_off, is_mads_enabled, is_mads_main_enabled

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


class FakeCarState:
  """The handful of CarState fields the comfort request reads, plus the car's AVH answer.

  Comfort_Control carries a direction rather than a toggle, so unlike the start-stop button it
  is not affected by the car's own frames interleaving with ours. It just answers, about 90 ms
  after the request lands.
  """

  AVH_ANSWER_DELAY = 9  # control frames before Comfort_Status carries the new state, ~90 ms measured

  def __init__(self, standstill=True, avh_active=None, stop_start_disabled=None, dashlights=True):
    self.out = SimpleNamespace(standstill=standstill)
    self.avh_active = avh_active
    self.stop_start_disabled = stop_start_disabled
    self.dashlights_msg = DASHLIGHTS_TEMPLATE if dashlights else None
    self.avh_deaf = False  # a car that hears the request but never acts on it
    self.avh_answer_in = None

  def step(self, sent):
    if any(addr == 0x6bb for addr, _, _ in sent):
      if not self.avh_deaf and self.avh_answer_in is None:
        self.avh_answer_in = self.AVH_ANSWER_DELAY
    if self.avh_answer_in is not None:
      self.avh_answer_in -= 1
      if self.avh_answer_in == 0:
        self.avh_active = True
        self.avh_answer_in = None


DASHLIGHTS_TEMPLATE = {"CHECKSUM": 0, "COUNTER": 3, "Signal1": 0, "Signal2": 0x11, "UNITS": 1,
                       "Signal3": 0x43, "ICY_ROAD": 0, "Signal4": 0x22, "Signal5": 0xfa,
                       "SEATBELT_FL": 1, "Signal6": 0, "LEFT_BLINKER": 0, "RIGHT_BLINKER": 0,
                       "Signal7": 0, "STOP_START": 0, "Signal8": 0, "Signal9": 0}


class FakeStopStartCar(FakeCarState):
  """A car that answers the start-stop button the way route 00000334 shows this one does.

  Dashlights goes out at 10 Hz with the button bit clear, and every one of those clear frames
  reads as a release. So the setting toggles once for each gap between the car's own frames
  that carries a press, no matter how many press frames landed in it. That is what turned an
  eight frame burst 50 ms apart into four presses and left the setting exactly where it
  started, which is the bug this models.
  """
  dashlights_msg: dict  # this car always has a frame to copy, unlike the bare fake above

  CAR_PERIOD = 10   # control frames between the car's own Dashlights frames, 100 Hz vs 10 Hz
  ANSWER_DELAY = 3  # control frames before Engine_Stop_Start carries the new state, ~30 ms measured

  def __init__(self, disabled=False, **kwargs):
    super().__init__(stop_start_disabled=disabled, **kwargs)
    self.dashlights_msg = dict(DASHLIGHTS_TEMPLATE)
    self.presses = 0
    self.gap_pressed = False
    self.deaf = False  # a car that counts presses but never acts on them
    self.answer_in = None
    self.tick = 0

  def step(self, sent):
    super().step(sent)
    if any(addr == 0x390 for addr, _, _ in sent):
      self.gap_pressed = True
    self.tick += 1
    if self.answer_in is not None:
      self.answer_in -= 1
      if self.answer_in == 0:
        self.stop_start_disabled = not self.stop_start_disabled
        self.answer_in = None
    if self.tick % self.CAR_PERIOD == 0:
      if self.gap_pressed:
        self.presses += 1
        if not self.deaf:
          self.answer_in = self.ANSWER_DELAY
        self.gap_pressed = False
      self.dashlights_msg["COUNTER"] = (self.dashlights_msg["COUNTER"] + 1) % 16


class TestSubaruComfort(unittest.TestCase):
  """Auto Vehicle Hold and the auto start-stop shutoff are start of drive requests.

  The car forgets both every ignition cycle. openpilot asks at the start of a drive, only if
  the car is in the wrong state, and repeats until the car answers. Once the car is in the
  wanted state that latches for the drive, so a touchscreen press always wins afterwards and a
  stuck state machine can never fight the driver.
  """
  PLATFORM = CAR.SUBARU_CROSSTREK_2024

  def _controller(self, avh=False, stop_start=False):
    CP = interfaces[self.PLATFORM].get_params(self.PLATFORM, {0: {}, 1: {}, 2: {}}, [], False, False, docs=False)
    if avh:
      enable_avh(CP)
    if stop_start:
      enable_stop_start_off(CP)
    return CarController(DBC[CP.carFingerprint], CP.as_reader())

  def _run(self, CC, CS, frames=3000, start=COMFORT_SETTLE_FRAMES):
    """Drive update_comfort directly over a window of frames and collect what it sent."""
    sent = []
    CC.frame = start
    for _ in range(frames):
      step = CC.update_comfort(CS)
      sent += step
      CS.step(step)
      CC.frame += 1
    return sent

  def test_nothing_without_either_flag(self):
    CC = self._controller()
    self.assertEqual([], self._run(CC, FakeCarState(avh_active=False, stop_start_disabled=False)))

  def test_avh_asks_once_when_off(self):
    CC = self._controller(avh=True)
    sent = self._run(CC, FakeCarState(avh_active=False))
    self.assertEqual(COMFORT_BURST_LEN, len(sent))
    for addr, dat, bus in sent:
      dat = bytes(dat)
      self.assertEqual(0x6bb, addr)
      self.assertEqual(CanBus.alt, bus)
      # 2 is the on request, and the rest of the frame is the steady state the panda pins
      self.assertEqual(2, dat[2])
      self.assertEqual(bytes([0x01, 0x00, 0x00, 0x0e, 0x00]), dat[3:])
      self.assertEqual((addr % 256 + addr // 256 + sum(dat[1:])) & 0xFF, dat[0])

  def test_avh_says_nothing_when_already_on(self):
    CC = self._controller(avh=True)
    self.assertEqual([], self._run(CC, FakeCarState(avh_active=True)))

  def test_avh_waits_for_a_real_message(self):
    # None means Comfort_Status has never arrived. asking on that is asking on a guess
    CC = self._controller(avh=True)
    self.assertEqual([], self._run(CC, FakeCarState(avh_active=None)))

  def test_stop_start_presses_once_when_armed(self):
    car = FakeStopStartCar(disabled=False)
    CC = self._controller(stop_start=True)
    sent = self._run(CC, car)
    self.assertEqual(COMFORT_PRESS_FRAMES, len(sent))
    for addr, dat, bus in sent:
      dat = bytes(dat)
      self.assertEqual(0x390, addr)
      self.assertEqual(CanBus.alt, bus)
      self.assertTrue(dat[6] & 0x40, "the button bit has to be set, the panda refuses it otherwise")
      # everything else is copied from the car's own frame, so the blinker and seatbelt bits stay true
      self.assertEqual(0x11, dat[2])
      self.assertEqual(1, dat[6] & 0x1)
      self.assertEqual((addr % 256 + addr // 256 + sum(dat[1:])) & 0xFF, dat[0])

  def test_stop_start_lands_as_exactly_one_press(self):
    # the regression. a press that straddles one of the car's own frames is two presses, and
    # an even number of presses leaves the shutoff exactly where it started.
    car = FakeStopStartCar(disabled=False)
    CC = self._controller(stop_start=True)
    self._run(CC, car)
    self.assertEqual(1, car.presses, "the car has to see one press, not a train of them")
    self.assertTrue(car.stop_start_disabled, "the shutoff has to end up off")

  def test_stop_start_lands_as_one_press_from_any_phase(self):
    # a two frame press only reads as one edge if it sits between two of the car's own
    # frames, so it is started off the car's counter rather than whenever we happen to be
    for phase in range(FakeStopStartCar.CAR_PERIOD):
      car = FakeStopStartCar(disabled=False)
      for _ in range(phase):
        car.step([])
      CC = self._controller(stop_start=True)
      self._run(CC, car)
      self.assertEqual(1, car.presses, f"{phase=}")
      self.assertTrue(car.stop_start_disabled, f"{phase=}")

  def test_stop_start_leaves_the_driver_alone_afterwards(self):
    # once the shutoff is off this is finished for the drive. a driver who presses the button
    # back on has to win, so a re-armed shutoff is never pressed a second time.
    car = FakeStopStartCar(disabled=False)
    CC = self._controller(stop_start=True)
    self._run(CC, car)
    self.assertTrue(car.stop_start_disabled)
    car.presses = 0
    car.stop_start_disabled = False
    self.assertEqual([], self._run(CC, car, start=CC.frame))
    self.assertEqual(0, car.presses)

  def test_stop_start_gives_up_after_a_few_tries(self):
    # the result is checked rather than assumed, so a press that does not land is repeated.
    # a car that never answers gets a bounded number of tries and is then left alone.
    car = FakeStopStartCar(disabled=False)
    car.deaf = True
    CC = self._controller(stop_start=True)
    sent = self._run(CC, car)
    self.assertEqual(COMFORT_PRESS_TRIES, car.presses)
    self.assertEqual(COMFORT_PRESS_TRIES * COMFORT_PRESS_FRAMES, len(sent))
    self.assertFalse(car.stop_start_disabled)

  def test_stop_start_counter_follows_the_car(self):
    car = FakeStopStartCar(disabled=False)
    CC = self._controller(stop_start=True)
    sent = self._run(CC, car)
    counters = [bytes(dat)[1] & 0xF for _, dat, _ in sent]
    self.assertEqual([(counters[0] + i) % 16 for i in range(len(counters))], counters)

  def test_stop_start_says_nothing_when_already_off(self):
    CC = self._controller(stop_start=True)
    self.assertEqual([], self._run(CC, FakeCarState(stop_start_disabled=True)))

  def test_stop_start_waits_for_a_frame_to_copy(self):
    CC = self._controller(stop_start=True)
    self.assertEqual([], self._run(CC, FakeCarState(stop_start_disabled=False, dashlights=False)))

  def test_avh_asks_while_moving(self):
    # the button arms the hold, it does not apply the brakes, and the car only ever holds once
    # it has already stopped. so this does not wait for a standstill either, and a driver who
    # pulls away before openpilot has started still gets the setting.
    CC = self._controller(avh=True)
    CS = FakeCarState(standstill=False, avh_active=False)
    self.assertEqual(COMFORT_BURST_LEN, len(self._run(CC, CS)))
    self.assertTrue(CS.avh_active)

  def test_avh_asks_again_if_the_first_one_missed(self):
    # a car that ignores the request would otherwise lose the setting for the whole drive
    CC = self._controller(avh=True)
    CS = FakeCarState(avh_active=False)
    CS.avh_deaf = True
    sent = self._run(CC, CS)
    self.assertGreater(len(sent), COMFORT_BURST_LEN, "one unanswered burst is not an answer")
    self.assertTrue(all(addr == 0x6bb for addr, _, _ in sent))

  def test_avh_leaves_the_driver_alone_afterwards(self):
    # once AVH has been seen on, openpilot is finished with it for the drive. a driver turning
    # it back off on the touchscreen is never fought.
    CC = self._controller(avh=True)
    CS = FakeCarState(avh_active=False)
    self.assertEqual(COMFORT_BURST_LEN, len(self._run(CC, CS)))
    self.assertTrue(CS.avh_active)
    CS.avh_active = False
    self.assertEqual([], self._run(CC, CS, frames=1000, start=CC.frame))

  def test_stop_start_asks_while_moving(self):
    # openpilot is often still starting up as the driver pulls away, and this button touches
    # nothing but the engine's own idle stop, so it does not wait for a standstill
    CC = self._controller(stop_start=True)
    car = FakeStopStartCar(disabled=False, standstill=False)
    self._run(CC, car)
    self.assertTrue(car.stop_start_disabled)

  def test_nothing_before_the_bus_settles(self):
    CC = self._controller(avh=True, stop_start=True)
    CS = FakeCarState(avh_active=False, stop_start_disabled=False)
    self.assertEqual([], self._run(CC, CS, frames=COMFORT_SETTLE_FRAMES, start=0))

  def test_gives_up_after_the_deadline(self):
    # past this it stops being a start of drive action, and surprising the driver with it
    # mid drive is worse than not doing it at all
    CC = self._controller(avh=True)
    CS = FakeCarState(avh_active=False)
    CS.avh_deaf = True
    self.assertNotEqual([], self._run(CC, CS, frames=COMFORT_DEADLINE_FRAMES))
    self.assertEqual([], self._run(CC, CS, start=CC.frame))

  def test_both_ask_together(self):
    CC = self._controller(avh=True, stop_start=True)
    car = FakeStopStartCar(disabled=False, avh_active=False)
    sent = self._run(CC, car)
    self.assertEqual(COMFORT_BURST_LEN, len([m for m in sent if m[0] == 0x6bb]))
    self.assertEqual(COMFORT_PRESS_FRAMES, len([m for m in sent if m[0] == 0x390]))
    self.assertTrue(car.stop_start_disabled)


if __name__ == "__main__":
  unittest.main()
