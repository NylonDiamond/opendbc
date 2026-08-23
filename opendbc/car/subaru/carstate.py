import copy
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.subaru.values import DBC, CanBus, CarControllerParams, SubaruFlags, is_mads_enabled, is_mads_main_enabled
from opendbc.car import CanSignalRateCalculator


class MadsLatch:
  """Mirrors the MADS latch in opendbc/safety/modes/subaru.h.

  openpilot's engaged state and the panda's controls_allowed have to agree. If openpilot
  reports disengaged while the panda still allows controls, the panda counts a heartbeat
  mismatch and clears controls_allowed after 3 seconds, which looks like steering randomly
  cutting out. test_subaru.py drives this and the C implementation with the same sequences
  to keep them in step.
  """

  def __init__(self):
    self.latched = False
    self.acc_enabled_prev = False
    # EyeSight main is already on when the car starts, so the first message we ever see
    # would otherwise look like a switch-on and arm during boot, before openpilot can
    # engage. start out assuming it is on, so only a real off/on tap arms.
    self.main_on_prev = True

  def update(self, main_on: bool, acc_enabled: bool, main_engage: bool) -> bool:
    # arm on the rising edge of ACC with the main switch on
    if main_on and acc_enabled and not self.acc_enabled_prev:
      self.latched = True
    # with main switch arming on, switching cruise on is enough by itself, no set speed
    if main_engage and main_on and not self.main_on_prev:
      self.latched = True
    # the main switch is the only thing that exits
    if not main_on:
      self.latched = False
    self.acc_enabled_prev = acc_enabled
    self.main_on_prev = main_on
    return self.latched


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["Transmission"]["Gear"]

    self.angle_rate_calulator = CanSignalRateCalculator(50)

    self.mads_latch = MadsLatch()

    # stays None on cars whose camera never sends the duplicate alert message
    self.es_lkas_alert_msg = None

    # comfort settings the car forgets every ignition cycle. all three stay None until a
    # real message has arrived, so a car that never sends them can never look like a car
    # sitting in the wrong state, and the one-shot request never fires on stale zeros.
    self.dashlights_msg = None
    self.avh_active = None
    self.stop_start_disabled = None

  @property
  def mads_enabled(self) -> bool:
    # MADS is configured by openpilot as a safety param, so the panda and this agree on it
    return is_mads_enabled(self.CP)

  @property
  def mads_main_enabled(self) -> bool:
    return is_mads_main_enabled(self.CP)

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    cp_alt = can_parsers[Bus.alt]
    ret = structs.CarState()

    throttle_msg = cp.vl["Throttle"] if not (self.CP.flags & SubaruFlags.HYBRID) else cp_alt.vl["Throttle_Hybrid"]
    ret.gasPressed = throttle_msg["Throttle_Pedal"] > 1e-5
    if self.CP.flags & SubaruFlags.PREGLOBAL:
      ret.brakePressed = cp.vl["Brake_Pedal"]["Brake_Pedal"] > 0
    else:
      cp_brakes = cp_alt if self.CP.flags & SubaruFlags.GLOBAL_GEN2 else cp
      ret.brakePressed = cp_brakes.vl["Brake_Status"]["Brake"] == 1

    cp_es_distance = cp_alt if self.CP.flags & (SubaruFlags.GLOBAL_GEN2 | SubaruFlags.HYBRID) else cp_cam
    if not (self.CP.flags & SubaruFlags.HYBRID):
      eyesight_fault = bool(cp_es_distance.vl["ES_Distance"]["Cruise_Fault"])

      # if openpilot is controlling long, an eyesight fault is a non-critical fault. otherwise it's an ACC fault
      if self.CP.openpilotLongitudinalControl:
        ret.carFaultedNonCritical = eyesight_fault
      else:
        ret.accFaulted = eyesight_fault

    cp_wheels = cp_alt if self.CP.flags & SubaruFlags.GLOBAL_GEN2 else cp
    self.parse_wheel_speeds(ret,
      cp_wheels.vl["Wheel_Speeds"]["FL"],
      cp_wheels.vl["Wheel_Speeds"]["FR"],
      cp_wheels.vl["Wheel_Speeds"]["RL"],
      cp_wheels.vl["Wheel_Speeds"]["RR"],
    )
    ret.standstill = ret.vEgoRaw == 0

    # continuous blinker signals for assisted lane change
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["Dashlights"]["LEFT_BLINKER"],
                                                                      cp.vl["Dashlights"]["RIGHT_BLINKER"])

    if self.CP.enableBsm:
      ret.leftBlindspot = (cp.vl["BSD_RCTA"]["L_ADJACENT"] == 1) or (cp.vl["BSD_RCTA"]["L_APPROACHING"] == 1)
      ret.rightBlindspot = (cp.vl["BSD_RCTA"]["R_ADJACENT"] == 1) or (cp.vl["BSD_RCTA"]["R_APPROACHING"] == 1)

    cp_transmission = cp_alt if self.CP.flags & SubaruFlags.HYBRID else cp
    can_gear = int(cp_transmission.vl["Transmission"]["Gear"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    if not (self.CP.flags & SubaruFlags.LKAS_ANGLE):
      ret.steeringAngleDeg = cp.vl["Steering_Torque"]["Steering_Angle"]
      steering_updated = len(cp.vl_all["Steering_Torque"]["Steering_Angle"]) > 0
    else:
      # Steering_Torque->Steering_Angle exists on SUBARU_FORESTER_2022, SUBARU_OUTBACK_2023, SUBARU_ASCENT_2023 where
      # it is identical to Steering_2's signal. However, it is always zero on newer LKAS_ANGLE cars
      # such as 2024+ Crosstrek, 2023+ Ascent, etc. Use a universal signal for LKAS_ANGLE cars.
      ret.steeringAngleDeg = cp.vl["Steering_2"]["Steering_Angle"]
      steering_updated = len(cp.vl_all["Steering_2"]["Steering_Angle"]) > 0

    if not (self.CP.flags & SubaruFlags.PREGLOBAL):
      # ideally we get this from the car, but unclear if it exists. diagnostic software doesn't even have it
      ret.steeringRateDeg = self.angle_rate_calulator.update(ret.steeringAngleDeg, steering_updated)

    ret.steeringTorque = cp.vl["Steering_Torque"]["Steer_Torque_Sensor"]
    ret.steeringTorqueEps = cp.vl["Steering_Torque"]["Steer_Torque_Output"]

    steer_threshold = 75 if self.CP.flags & SubaruFlags.PREGLOBAL else 80
    ret.steeringPressed = abs(ret.steeringTorque) > steer_threshold

    cp_cruise = cp_alt if self.CP.flags & SubaruFlags.GLOBAL_GEN2 else cp
    cp_es_brake = cp_alt if self.CP.flags & SubaruFlags.GLOBAL_GEN2 else cp_cam

    # brake commanded by eyesight, for the UI. meaningless when openpilot controls long,
    # since ES_Brake is then openpilot's own output echoed back. preglobal is excluded
    # because its ES_Brake has a different layout and BRAKE_MAX does not apply to it.
    if not (self.CP.flags & SubaruFlags.PREGLOBAL) and not self.CP.openpilotLongitudinalControl:
      brake_pressure = cp_es_brake.vl["ES_Brake"]["Brake_Pressure"]
      ret.stockBrakeCommand = min(1.0, max(0.0, brake_pressure / CarControllerParams.BRAKE_MAX))

    if self.CP.flags & SubaruFlags.LKAS_ANGLE:
      # ES_Brake->Cruise_Activated stays high on brake at standstill,
      # so we use ES_Status->Cruise_Activated which is the correct engaged state.
      acc_enabled = cp_es_brake.vl["ES_Status"]['Cruise_Activated'] != 0
      main_on = cp_cam.vl["ES_DashStatus"]['Cruise_On'] != 0
      ret.cruiseState.available = main_on

      ret.stockCruiseEngaged = acc_enabled
      if self.mads_enabled:
        ret.cruiseState.enabled = self.mads_latch.update(main_on, acc_enabled, self.mads_main_enabled)
      else:
        ret.cruiseState.enabled = acc_enabled
    elif self.CP.flags & SubaruFlags.HYBRID:
      # ES_Status is missing on hybrid, so we use ES_Brake instead
      # TODO: 0x27 and 0x225 on hybrids may work as a replacement
      ret.cruiseState.enabled = cp_es_brake.vl["ES_Brake"]['Cruise_Activated'] != 0
      ret.cruiseState.available = cp_cam.vl["ES_DashStatus"]['Cruise_On'] != 0
      ret.stockCruiseEngaged = ret.cruiseState.enabled
    else:
      ret.cruiseState.enabled = cp_cruise.vl["CruiseControl"]["Cruise_Activated"] != 0
      ret.cruiseState.available = cp_cruise.vl["CruiseControl"]["Cruise_On"] != 0
      ret.stockCruiseEngaged = ret.cruiseState.enabled
    ret.cruiseState.speed = cp_cam.vl["ES_DashStatus"]["Cruise_Set_Speed"] * CV.KPH_TO_MS

    if (self.CP.flags & SubaruFlags.PREGLOBAL and cp.vl["Dash_State2"]["UNITS"] == 1) or \
       (not (self.CP.flags & SubaruFlags.PREGLOBAL) and cp.vl["Dashlights"]["UNITS"] == 1):
      ret.cruiseState.speed *= CV.MPH_TO_KPH

    # the dash keeps its last set speed after ACC drops, so with MADS holding lateral the
    # cluster would show a number nothing is acting on. report no set speed instead, which
    # is what the driver needs to know: openpilot is steering and the pedals are theirs.
    # only while engaged, so a disengaged openpilot still shows the speed ACC would resume at.
    if self.mads_enabled and ret.cruiseState.enabled and not ret.stockCruiseEngaged:
      ret.cruiseState.speed = 0.

    ret.seatbeltUnlatched = cp.vl["Dashlights"]["SEATBELT_FL"] == 1
    ret.doorOpen = any([cp.vl["BodyInfo"]["DOOR_OPEN_RR"],
                        cp.vl["BodyInfo"]["DOOR_OPEN_RL"],
                        cp.vl["BodyInfo"]["DOOR_OPEN_FR"],
                        cp.vl["BodyInfo"]["DOOR_OPEN_FL"]])
    ret.steerFaultPermanent = cp.vl["Steering_Torque"]["Steer_Error_1"] == 1

    if self.CP.flags & SubaruFlags.PREGLOBAL:
      self.cruise_button = cp_cam.vl["ES_Distance"]["Cruise_Button"]
      self.ready = not cp_cam.vl["ES_DashStatus"]["Not_Ready_Startup"]
    else:
      ret.steerFaultTemporary = cp.vl["Steering_Torque"]["Steer_Warning"] == 1
      ret.cruiseState.nonAdaptive = cp_cam.vl["ES_DashStatus"]["Conventional_Cruise"] == 1
      ret.cruiseState.standstill = cp_cam.vl["ES_DashStatus"]["Cruise_State"] == 3
      ret.stockFcw = (cp_cam.vl["ES_LKAS_State"]["LKAS_Alert"] == 1) or \
                     (cp_cam.vl["ES_LKAS_State"]["LKAS_Alert"] == 2)

      self.es_lkas_state_msg = copy.copy(cp_cam.vl["ES_LKAS_State"])

      # Not every camera sends the duplicate alert message, so only claim it once one has
      # actually arrived. ts_nanos stays 0 until the first receipt, and sending a fabricated
      # copy on a car that never had it would put a message on the bus that does not belong.
      # vl has to be read first: it registers the message with the parser, ts_nanos does not.
      es_lkas_alert = cp_cam.vl["ES_LKAS_Alert"]
      if cp_cam.ts_nanos["ES_LKAS_Alert"]["LKAS_Alert_Msg"] != 0:
        self.es_lkas_alert_msg = copy.copy(es_lkas_alert)
      self.es_brake_msg = copy.copy(cp_es_brake.vl["ES_Brake"])

      # TODO: Hybrid cars don't have ES_Distance, need a replacement
      if not (self.CP.flags & SubaruFlags.HYBRID):
        # 8 is known AEB, there are a few other values related to AEB we ignore
        ret.stockAeb = (cp_es_distance.vl["ES_Brake"]["AEB_Status"] == 8) and \
                       (cp_es_distance.vl["ES_Brake"]["Brake_Pressure"] != 0)

        self.es_status_msg = copy.copy(cp_es_brake.vl["ES_Status"])
        self.cruise_control_msg = copy.copy(cp_cruise.vl["CruiseControl"])

    if not (self.CP.flags & SubaruFlags.HYBRID):
      self.es_distance_msg = copy.copy(cp_es_distance.vl["ES_Distance"])

    self.es_dashstatus_msg = copy.copy(cp_cam.vl["ES_DashStatus"])
    if self.CP.flags & SubaruFlags.SEND_INFOTAINMENT:
      self.es_infotainment_msg = copy.copy(cp_cam.vl["ES_Infotainment"])

    # *** comfort settings, read from the main bus and requested on the alt bus ***
    # only on the gen2 angle cars these messages were measured on, which is also the only
    # place the panda will let the requests out. preglobal cars use a different DBC that has
    # none of these signals at all, so reading them there is a KeyError, not a zero.
    if self.CP.flags & SubaruFlags.GLOBAL_GEN2 and self.CP.flags & SubaruFlags.LKAS_ANGLE:
      # vl has to be read before ts_nanos, which is what registers the message with the
      # parser. a message the car never sends keeps ts_nanos at 0 and leaves these None.
      dashlights = cp.vl["Dashlights"]
      if cp.ts_nanos["Dashlights"]["STOP_START"] != 0:
        self.dashlights_msg = copy.copy(dashlights)

      comfort_status = cp.vl["Comfort_Status"]
      if cp.ts_nanos["Comfort_Status"]["AVH_ACTIVE"] != 0:
        self.avh_active = comfort_status["AVH_ACTIVE"] == 1

      stop_start = cp.vl["Engine_Stop_Start"]
      if cp.ts_nanos["Engine_Stop_Start"]["STOP_START_STATE"] != 0:
        # 0 is the shutoff armed and ready, 3 is it switched off. 2 shows up briefly at boot
        self.stop_start_disabled = stop_start["STOP_START_STATE"] == 3

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus.main),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus.camera),
      Bus.alt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus.alt)
    }
