import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, make_tester_present_msg
from opendbc.car.lateral import apply_center_deadzone, apply_driver_steer_torque_limits, apply_steer_angle_limits_vm, common_fault_avoidance
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.subaru import subarucan
from opendbc.car.subaru.values import DBC, GLOBAL_ES_ADDR, CanBus, CarControllerParams, SubaruFlags, \
                                      is_avh_enabled, is_stop_start_off_enabled
from opendbc.car.vehicle_model import VehicleModel

# FIXME: These limits aren't exact. The real limit is more than likely over a larger time period and
# involves the total steering angle change rather than rate, but these limits work well for now
MAX_STEER_RATE = 25  # deg/s
MAX_STEER_RATE_FRAMES = 7  # tx control frames needed before torque can be cut

# Comfort settings the car forgets every ignition cycle. Each request is made at the start of a
# drive, only if the car is in the wrong state, repeated until the car answers, and then latched
# done for the rest of the drive. Neither waits for standstill. Frames are 100 Hz.
COMFORT_SETTLE_FRAMES = 500   # 5 s, long enough for every message we compare against to arrive
COMFORT_DEADLINE_FRAMES = 6000  # 60 s, after which this stops being a start of drive action
COMFORT_BURST_LEN = 8         # frames per request, matching a real button press
COMFORT_BURST_STEP = 5        # 50 ms apart, also matching
# 2 s between attempts. the car answers in about 90 ms, so this is mostly about not hammering it
COMFORT_BURST_SETTLE = 200

# The start-stop button is edge triggered, and the car keeps sending its own Dashlights at
# 10 Hz with the button bit clear. Every one of those clear frames reads as a release, so a
# burst that spans several of them lands as several presses, and the setting toggles back and
# forth. Measured on route 00000334: eight frames 50 ms apart became four presses and ended
# up exactly where it started. So a press is kept short enough to sit inside one 100 ms gap
# between the car's own frames, and it is checked and repeated instead of assumed.
COMFORT_PRESS_FRAMES = 2      # 20 ms of button bit, well inside one gap
COMFORT_PRESS_SETTLE = 40     # 400 ms to let the car answer before judging the press
COMFORT_PRESS_TRIES = 5       # give up rather than sit here toggling


def get_safety_CP():
  # Use the Ascent for lateral limiting to match safety (most restrictive slip factor)
  from opendbc.car.subaru.interface import CarInterface
  return CarInterface.get_non_essential_params("SUBARU_ASCENT")


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_torque_last = 0
    self.apply_angle_last = 0

    self.cruise_button_prev = 0
    self.steer_rate_counter = 0

    # start of drive comfort requests: frames still to send, how long to wait for the car to
    # answer, and whether this drive is finished with them either way. once done latches, the
    # driver is left alone for the rest of the drive.
    self.avh_burst_left = 0
    self.avh_counter = 0
    self.avh_wait = 0
    self.avh_done = False
    self.stop_start_press_left = 0
    self.stop_start_wait = 0
    self.stop_start_tries = 0
    self.stop_start_counter = 0
    self.stop_start_seen_counter = None
    self.stop_start_done = False

    self.p = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint][Bus.pt])

    if CP.flags & SubaruFlags.LKAS_ANGLE:
      self.VM = VehicleModel(get_safety_CP())

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel

    can_sends = []

    # *** steering ***
    if (self.frame % self.p.STEER_STEP) == 0:
      if self.CP.flags & SubaruFlags.LKAS_ANGLE:
        apply_angle = actuators.steeringAngleDeg
        # heavy steering oscillation at low speeds (up to ~5 mph), still present up to ~22mph
        # likely due to poor steering angle sensor resolution or imprecise EPS actuation
        if CC.latActive and CS.out.vEgoRaw < 10.0:
          deadzone = np.interp(CS.out.vEgoRaw, [2., 10.0], [6.0, 3.0])
          apply_angle = self.apply_angle_last + apply_center_deadzone(apply_angle - self.apply_angle_last, deadzone)
        self.apply_angle_last = apply_steer_angle_limits_vm(apply_angle, self.apply_angle_last, CS.out.vEgoRaw,
                                                            CS.out.steeringAngleDeg, CC.latActive, CarControllerParams, self.VM)
        can_sends.append(subarucan.create_steering_control_angle(self.packer, self.apply_angle_last, CC.latActive))
      else:
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
      apply_throttle = int(round(np.interp(actuators.accel, CarControllerParams.THROTTLE_LOOKUP_BP, CarControllerParams.THROTTLE_LOOKUP_V)))
      apply_rpm = int(round(np.interp(actuators.accel, CarControllerParams.RPM_LOOKUP_BP, CarControllerParams.RPM_LOOKUP_V)))
      apply_brake = int(round(np.interp(actuators.accel, CarControllerParams.BRAKE_LOOKUP_BP, CarControllerParams.BRAKE_LOOKUP_V)))

      # limit min and max values
      cruise_throttle = np.clip(apply_throttle, CarControllerParams.THROTTLE_MIN, CarControllerParams.THROTTLE_MAX)
      cruise_rpm = np.clip(apply_rpm, CarControllerParams.RPM_MIN, CarControllerParams.RPM_MAX)
      cruise_brake = np.clip(apply_brake, CarControllerParams.BRAKE_MIN, CarControllerParams.BRAKE_MAX)
    else:
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
        can_sends.append(subarucan.create_es_dashstatus(self.packer, self.frame // 10, CS.es_dashstatus_msg, CC.enabled,
                                                        self.CP.openpilotLongitudinalControl, CC.longActive, hud_control.leadVisible))

        can_sends.append(subarucan.create_es_lkas_state(self.packer, self.frame // 10, CS.es_lkas_state_msg, CC.enabled, hud_control.visualAlert,
                                                        hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                                        hud_control.leftLaneDepart, hud_control.rightLaneDepart))

        if self.CP.flags & SubaruFlags.SEND_INFOTAINMENT:
          can_sends.append(subarucan.create_es_infotainment(self.packer, self.frame // 10, CS.es_infotainment_msg, hud_control.visualAlert))

        # the panda blocks the camera's own copy, so this has to replace it or the dash
        # loses the alert message entirely. only sent once we have seen one to copy
        if CS.es_lkas_alert_msg is not None:
          can_sends.append(subarucan.create_es_lkas_alert(self.packer, self.frame // 10, CS.es_lkas_alert_msg, hud_control.visualAlert))

      if self.CP.openpilotLongitudinalControl:
        if self.frame % 5 == 0:
          can_sends.append(subarucan.create_es_status(self.packer, self.frame // 5, CS.es_status_msg,
                                                      self.CP.openpilotLongitudinalControl, CC.longActive, cruise_rpm))

          can_sends.append(subarucan.create_es_brake(self.packer, self.frame // 5, CS.es_brake_msg,
                                                     self.CP.openpilotLongitudinalControl, CC.longActive, cruise_brake))

          can_sends.append(subarucan.create_es_distance(self.packer, self.frame // 5, CS.es_distance_msg, 0, pcm_cancel_cmd,
                                                        self.CP.openpilotLongitudinalControl, cruise_brake > 0, cruise_throttle))
      else:
        if pcm_cancel_cmd:
          if not (self.CP.flags & SubaruFlags.HYBRID):
            bus = CanBus.alt if self.CP.flags & SubaruFlags.GLOBAL_GEN2 else CanBus.main
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

    can_sends += self.update_comfort(CS)

    new_actuators = actuators.as_builder()
    if self.CP.flags & SubaruFlags.LKAS_ANGLE:
      new_actuators.steeringAngleDeg = self.apply_angle_last
    else:
      new_actuators.torque = self.apply_torque_last / self.p.STEER_MAX
      new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends

  def update_comfort(self, CS):
    """Set Auto Vehicle Hold and the auto start-stop shutoff once, at the start of a drive.

    The car forgets both every ignition cycle, so each request only ever pushes in the one
    direction the driver had to push it by hand: AVH on, start-stop off. openpilot can never
    switch AVH off or switch start-stop back on, which keeps a stuck state machine from
    undoing something the driver just did on the touchscreen.
    """
    can_sends = []

    avh_wanted = is_avh_enabled(self.CP)
    stop_start_wanted = is_stop_start_off_enabled(self.CP)
    if not (avh_wanted or stop_start_wanted):
      return can_sends

    # let the bus settle first, then give up if the car never answered. after the deadline this
    # stops being a start of drive action, and surprising the driver with it mid drive is worse
    # than not doing it at all.
    if self.frame < COMFORT_SETTLE_FRAMES:
      return can_sends
    if self.frame > COMFORT_DEADLINE_FRAMES:
      self.avh_done = True
      self.stop_start_done = True
      return can_sends

    # *** auto vehicle hold ***
    # not gated on standstill. the button arms the hold, it does not apply the brakes: the car
    # only ever holds once it has already stopped under the driver's own braking. so asking
    # while rolling does exactly what the driver's own thumb does, and openpilot is often still
    # starting up as the driver pulls away, so a standstill gate here mostly just missed.
    #
    # the result is read back and the request repeated, rather than fired once and assumed. a
    # car that ignores this while moving would otherwise lose the setting for the whole drive.
    # the moment AVH is seen on this latches done, so a later touchscreen press off is never
    # fought, and the retry loop only ever runs while AVH has never once been on.
    if avh_wanted and not self.avh_done:
      if self.avh_burst_left > 0:
        # mid burst. a real press keeps going after the car has already answered, so this one
        # runs to the end rather than cutting short the moment the state flips.
        if self.frame % COMFORT_BURST_STEP == 0:
          self.avh_counter += 1
          # 2 is the on request. the car answers on Comfort_Status about 90 ms later
          can_sends.append(subarucan.create_comfort_control(self.packer, self.avh_counter, 2))
          self.avh_burst_left -= 1
          if self.avh_burst_left == 0:
            self.avh_wait = COMFORT_BURST_SETTLE
      elif CS.avh_active:
        # on, either already or because a burst took. never ask again this drive
        self.avh_done = True
      elif self.avh_wait > 0:
        # give the car room to answer before deciding the request missed
        self.avh_wait -= 1
      elif CS.avh_active is False:
        # a definite reading of off. None means Comfort_Status has not arrived yet
        self.avh_burst_left = COMFORT_BURST_LEN

    # *** auto start-stop engine shutoff ***
    # this one is a button press rather than a state, so it toggles. only ever press it when
    # the shutoff is still armed, or it would switch the thing back on.
    #
    # unlike AVH this does not wait for standstill. it touches nothing but the engine's own
    # idle stop, and openpilot is often still starting up as the driver pulls away, so a
    # standstill gate here would mostly just miss.
    if stop_start_wanted and not self.stop_start_done and CS.dashlights_msg is not None:
      # the car's counter advances once per Dashlights frame, so a change here means one of
      # its frames just landed and the gap before the next one is ours
      car_counter = int(CS.dashlights_msg["COUNTER"])
      fresh_frame = self.stop_start_seen_counter is not None and car_counter != self.stop_start_seen_counter
      self.stop_start_seen_counter = car_counter

      if CS.stop_start_disabled:
        # the shutoff is off, which is all we wanted
        self.stop_start_done = True
      elif self.stop_start_press_left > 0:
        # mid press. hold the button bit down on consecutive frames so the car sees one edge
        self.stop_start_counter += 1
        can_sends.append(subarucan.create_stop_start_press(self.packer, self.stop_start_counter,
                                                           CS.dashlights_msg))
        self.stop_start_press_left -= 1
        if self.stop_start_press_left == 0:
          self.stop_start_wait = COMFORT_PRESS_SETTLE
      elif self.stop_start_wait > 0:
        # the car answers on Engine_Stop_Start within about 30 ms, but give it room
        self.stop_start_wait -= 1
      elif self.stop_start_tries >= COMFORT_PRESS_TRIES:
        # something about this car does not match what was measured. stop rather than sit
        # here toggling the setting for the rest of the drive.
        self.stop_start_done = True
      elif CS.stop_start_disabled is False and fresh_frame:
        # start the press right after one of the car's own frames, so the whole press fits in
        # the gap before the next one and reads as a single edge
        self.stop_start_counter = car_counter
        self.stop_start_tries += 1
        self.stop_start_press_left = COMFORT_PRESS_FRAMES

    return can_sends
