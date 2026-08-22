import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, make_tester_present_msg
from opendbc.car.lateral import apply_center_deadzone, apply_driver_steer_torque_limits, apply_steer_angle_limits_vm, common_fault_avoidance
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.subaru import subarucan
from opendbc.car.subaru.values import DBC, GLOBAL_ES_ADDR, CanBus, CarControllerParams, SubaruFlags, is_auto_resume_enabled
from opendbc.car.vehicle_model import VehicleModel

# FIXME: These limits aren't exact. The real limit is more than likely over a larger time period and
# involves the total steering angle change rather than rate, but these limits work well for now
MAX_STEER_RATE = 25  # deg/s
MAX_STEER_RATE_FRAMES = 7  # tx control frames needed before torque can be cut

# Auto resume out of EyeSight's stay stopped state, which the dash shows as "HOLD". Faking a
# press of the resume button is the only way back out: this car can never have openpilot
# longitudinal, and the steering wheel buttons ride a message the panda will not let openpilot
# send. Values measured off a real resume: the button pulse is 3 sends of ES_Distance at 20 Hz,
# and Close_Distance sits in a +/- 0.2 m noise band while stopped.
RESUME_PULSE_SENDS = 3       # 150 ms, matching a real press
RESUME_LEAD_DELTA = 0.5      # m the lead must gain on us, 2.5x the noise band
RESUME_TRIGGER_FRAMES = 15   # 150 ms of the gap staying open, so one noisy sample cannot fire it
RESUME_MIN_HOLD_FRAMES = 50  # 0.5 s settled in HOLD before arming, so a rolling stop cannot trip it
RESUME_RETRY_FRAMES = 150    # 1.5 s before trying again, in case the car ignored the first press
RESUME_MAX_ATTEMPTS = 3


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

    self.resume_sends_left = 0
    self.resume_lead_min = 0.0
    self.resume_lead_seen = False
    self.resume_hold_frames = 0
    self.resume_trigger_frames = 0
    self.resume_attempts = 0
    self.resume_wait_frames = 0

    self.p = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint][Bus.pt])

    if CP.flags & SubaruFlags.LKAS_ANGLE:
      self.VM = VehicleModel(get_safety_CP())

  @property
  def auto_resume_available(self) -> bool:
    """Whether faking the resume button is allowed at all.

    Read live, never snapshot: the safety flag is applied after the interface is built, so
    anything caching this in __init__ gets a stale answer. Hybrid and preglobal cars are out
    because they have no ES_Distance in the shape this uses.
    """
    return (not self.CP.openpilotLongitudinalControl and
            not (self.CP.flags & (SubaruFlags.PREGLOBAL | SubaruFlags.HYBRID)) and
            is_auto_resume_enabled(self.CP))

  def _reset_auto_resume(self) -> None:
    self.resume_sends_left = 0
    self.resume_lead_seen = False
    self.resume_hold_frames = 0
    self.resume_trigger_frames = 0
    self.resume_attempts = 0
    self.resume_wait_frames = 0

  def _update_auto_resume(self, CC, CS) -> bool:
    """Decide whether a faked resume press is wanted this frame.

    Fires only from a standstill in EyeSight's stay stopped state, and only when a lead we
    were actually following has pulled away. No lead means no resume: sitting first in the
    queue at a red light has to stay the driver's call, since nothing on this car can tell
    us the light changed. Stock ACC still owns the pedals once it takes the request, so it
    will stop again on its own if something is in the way.
    """
    if not self.auto_resume_available:
      self._reset_auto_resume()
      return False

    # Cruise_State 3 is HOLD. State 1 is HOLD with the driver on the brake, which is
    # deliberately excluded: it is the driver holding the car, not EyeSight.
    in_hold = CS.out.cruiseState.standstill and CS.out.standstill and CS.out.stockCruiseEngaged
    driver_override = CS.out.brakePressed or CS.out.gasPressed

    if not (in_hold and CC.enabled and not driver_override):
      self._reset_auto_resume()
      return False

    self.resume_hold_frames += 1
    if self.resume_wait_frames > 0:
      self.resume_wait_frames -= 1

    close_distance = CS.es_distance_msg["Close_Distance"]
    if CS.es_distance_msg["Car_Follow"]:
      self.resume_lead_min = close_distance if not self.resume_lead_seen else min(self.resume_lead_min, close_distance)
      self.resume_lead_seen = True

    # Car_Follow is not required to still be set here. A lead that pulls away hard drops it
    # while the gap is wide open, and that is exactly the case worth resuming for.
    lead_moved = self.resume_lead_seen and (close_distance - self.resume_lead_min) > RESUME_LEAD_DELTA
    armed = (self.resume_hold_frames > RESUME_MIN_HOLD_FRAMES and
             self.resume_wait_frames == 0 and
             self.resume_attempts < RESUME_MAX_ATTEMPTS)

    self.resume_trigger_frames = self.resume_trigger_frames + 1 if (lead_moved and armed) else 0

    if self.resume_trigger_frames >= RESUME_TRIGGER_FRAMES and self.resume_sends_left == 0:
      self.resume_sends_left = RESUME_PULSE_SENDS
      self.resume_attempts += 1
      self.resume_wait_frames = RESUME_RETRY_FRAMES
      self.resume_trigger_frames = 0

    # 20 Hz, the rate the camera sends ES_Distance at, so the pulse lasts as long as a real press
    send_resume = self.resume_sends_left > 0 and self.frame % 5 == 0
    if send_resume:
      self.resume_sends_left -= 1
    return send_resume

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
        # runs every frame so the hold timers stay honest, whatever the send path does
        cruise_resume_cmd = self._update_auto_resume(CC, CS)

        if pcm_cancel_cmd:
          if not (self.CP.flags & SubaruFlags.HYBRID):
            bus = CanBus.alt if self.CP.flags & SubaruFlags.GLOBAL_GEN2 else CanBus.main
            can_sends.append(subarucan.create_es_distance(self.packer, CS.es_distance_msg["COUNTER"] + 1, CS.es_distance_msg, bus, pcm_cancel_cmd))
        elif cruise_resume_cmd:
          bus = CanBus.alt if self.CP.flags & SubaruFlags.GLOBAL_GEN2 else CanBus.main
          can_sends.append(subarucan.create_es_distance(self.packer, CS.es_distance_msg["COUNTER"] + 1, CS.es_distance_msg, bus, False,
                                                        cruise_resume_cmd=True))

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

    new_actuators = actuators.as_builder()
    if self.CP.flags & SubaruFlags.LKAS_ANGLE:
      new_actuators.steeringAngleDeg = self.apply_angle_last
    else:
      new_actuators.torque = self.apply_torque_last / self.p.STEER_MAX
      new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends
