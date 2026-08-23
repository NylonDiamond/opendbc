#pragma once

#include "opendbc/safety/declarations.h"

#define SUBARU_STEERING_LIMITS_GENERATOR(steer_max, rate_up, rate_down)               \
  {                                                                                   \
    .max_torque = (steer_max),                                                        \
    .max_rt_delta = 940,                                                              \
    .max_rate_up = (rate_up),                                                         \
    .max_rate_down = (rate_down),                                                     \
    .driver_torque_multiplier = 50,                                                   \
    .driver_torque_allowance = 60,                                                    \
    .type = TorqueDriverLimited,                                                      \
    /* the EPS will temporary fault if the steering rate is too high, so we cut the   \
       the steering torque every 7 frames for 1 frame if the steering rate is high */ \
    .min_valid_request_frames = 7,                                                    \
    .max_invalid_request_frames = 1,                                                  \
    .min_valid_request_rt_interval = 144000,  /* 10% tolerance */                     \
    .has_steer_req_tolerance = true,                                                  \
  }

#define MSG_SUBARU_Brake_Status          0x13cU
#define MSG_SUBARU_CruiseControl         0x240U
#define MSG_SUBARU_Throttle              0x40U
#define MSG_SUBARU_Steering_Torque       0x119U
#define MSG_SUBARU_Steering_2            0x11aU
#define MSG_SUBARU_Wheel_Speeds          0x13aU
#define MSG_SUBARU_Dashlights            0x390U
#define MSG_SUBARU_Comfort_Control       0x6bbU

#define MSG_SUBARU_ES_LKAS               0x122U
#define MSG_SUBARU_ES_LKAS_ANGLE         0x124U
#define MSG_SUBARU_ES_Brake              0x220U
#define MSG_SUBARU_ES_Distance           0x221U
#define MSG_SUBARU_ES_Status             0x222U
#define MSG_SUBARU_ES_DashStatus         0x321U
#define MSG_SUBARU_ES_LKAS_State         0x322U
#define MSG_SUBARU_ES_Infotainment       0x323U
#define MSG_SUBARU_ES_LKAS_Alert         0x3C4U

#define MSG_SUBARU_ES_UDS_Request        0x787U

#define MSG_SUBARU_ES_HighBeamAssist     0x22AU
#define MSG_SUBARU_ES_STATIC_1           0x325U
#define MSG_SUBARU_ES_STATIC_2           0x121U

#define SUBARU_MAIN_BUS 0U
#define SUBARU_ALT_BUS  1U
#define SUBARU_CAM_BUS  2U

#define SUBARU_BASE_TX_MSGS(alt_bus, lkas_msg) \
  {lkas_msg,                     SUBARU_MAIN_BUS, 8, .check_relay = true},  \
  {MSG_SUBARU_ES_DashStatus,     SUBARU_MAIN_BUS, 8, .check_relay = true},  \
  {MSG_SUBARU_ES_LKAS_State,     SUBARU_MAIN_BUS, 8, .check_relay = true},  \
  {MSG_SUBARU_ES_Infotainment,   SUBARU_MAIN_BUS, 8, .check_relay = true},  \

/* The camera on LKAS_ANGLE cars repeats its LKAS alert on a second address. openpilot has
   to replace that one too, so the stock "Keep hands on wheel" nag stays off the dash while
   openpilot is steering. Cameras that never send it simply have nothing to block. */
#define SUBARU_LKAS_ALERT_TX_MSGS() \
  {MSG_SUBARU_ES_LKAS_Alert,     SUBARU_MAIN_BUS, 8, .check_relay = true},  \

#define SUBARU_COMMON_TX_MSGS(alt_bus) \
  {MSG_SUBARU_ES_Distance, alt_bus, 8, .check_relay = false}, \

/* Auto Vehicle Hold and the auto start-stop shutoff, both of which the car forgets every
   ignition cycle. The car sends both messages itself, so relay checking would fault; what
   keeps these safe is the tx hook, which refuses them unless openpilot was configured to
   ask for them and refuses them outright while the car is moving. */
#define SUBARU_COMFORT_TX_MSGS(alt_bus) \
  {MSG_SUBARU_Comfort_Control,   alt_bus,         8, .check_relay = false}, \
  {MSG_SUBARU_Dashlights,        alt_bus,         8, .check_relay = false}, \

#define SUBARU_COMMON_LONG_TX_MSGS(alt_bus) \
  {MSG_SUBARU_ES_Distance,       alt_bus,         8, .check_relay = true}, \
  {MSG_SUBARU_ES_Brake,          alt_bus,         8, .check_relay = true}, \
  {MSG_SUBARU_ES_Status,         alt_bus,         8, .check_relay = true}, \

#define SUBARU_GEN2_LONG_ADDITIONAL_TX_MSGS() \
  {MSG_SUBARU_ES_UDS_Request,    SUBARU_CAM_BUS,  8, .check_relay = false}, \
  {MSG_SUBARU_ES_HighBeamAssist, SUBARU_MAIN_BUS, 8, .check_relay = false}, \
  {MSG_SUBARU_ES_STATIC_1,       SUBARU_MAIN_BUS, 8, .check_relay = false}, \
  {MSG_SUBARU_ES_STATIC_2,       SUBARU_MAIN_BUS, 8, .check_relay = false}, \

#define SUBARU_COMMON_RX_CHECKS(alt_bus)                                                                                                         \
  {.msg = {{MSG_SUBARU_Throttle,        SUBARU_MAIN_BUS, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
  {.msg = {{MSG_SUBARU_Steering_Torque, SUBARU_MAIN_BUS, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_SUBARU_Wheel_Speeds,    alt_bus,         8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_SUBARU_Brake_Status,    alt_bus,         8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_SUBARU_CruiseControl,   alt_bus,         8, 20U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

#define SUBARU_LKAS_ANGLE_RX_CHECKS(alt_bus)                                                                                                    \
  {.msg = {{MSG_SUBARU_Throttle,        SUBARU_MAIN_BUS, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
  {.msg = {{MSG_SUBARU_Steering_Torque, SUBARU_MAIN_BUS, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_SUBARU_Wheel_Speeds,    alt_bus,         8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_SUBARU_Brake_Status,    alt_bus,         8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_SUBARU_ES_Status,       alt_bus,         8, 20U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_SUBARU_Steering_2,      SUBARU_MAIN_BUS, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

/* MADS additionally needs the main switch, which is only carried by the camera's own
   ES_DashStatus on the camera bus. The copy we transmit on the main bus is our own output,
   so it cannot be used to decide whether controls are allowed. */
#define SUBARU_LKAS_ANGLE_MADS_RX_CHECKS(alt_bus)                                                                                               \
  SUBARU_LKAS_ANGLE_RX_CHECKS(alt_bus)                                                                                                          \
  {.msg = {{MSG_SUBARU_ES_DashStatus,   SUBARU_CAM_BUS,  8, 10U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

static bool subaru_gen2 = false;
static bool subaru_longitudinal = false;
static bool subaru_lkas_angle = false;
static bool subaru_mads = false;
static bool subaru_mads_main = false;
static bool subaru_main_on_prev = true;
static bool subaru_avh = false;
static bool subaru_stop_start = false;

static uint32_t subaru_get_checksum(const CANPacket_t *msg) {
  return (uint8_t)msg->data[0];
}

static uint8_t subaru_get_counter(const CANPacket_t *msg) {
  return (uint8_t)(msg->data[1] & 0xFU);
}

static uint32_t subaru_compute_checksum(const CANPacket_t *msg) {
  int len = GET_LEN(msg);
  uint8_t checksum = (uint8_t)(msg->addr) + (uint8_t)((unsigned int)(msg->addr) >> 8U);
  for (int i = 1; i < len; i++) {
    checksum += (uint8_t)msg->data[i];
  }
  return checksum;
}

static void subaru_rx_hook(const CANPacket_t *msg) {
  const unsigned int alt_main_bus = subaru_gen2 ? SUBARU_ALT_BUS : SUBARU_MAIN_BUS;

  if ((msg->addr == MSG_SUBARU_Steering_Torque) && (msg->bus == SUBARU_MAIN_BUS)) {
    int torque_driver_new;
    torque_driver_new = ((GET_BYTES(msg, 0, 4) >> 16) & 0x7FFU);
    torque_driver_new = -1 * to_signed(torque_driver_new, 11);
    update_sample(&torque_driver, torque_driver_new);
  }

  // Steering angle measurement for LKAS_ANGLE cars
  if (subaru_lkas_angle && (msg->addr == MSG_SUBARU_Steering_2) && (msg->bus == SUBARU_MAIN_BUS)) {
    int angle_meas_new = GET_BYTES(msg, 3, 3) & 0x1FFFFU;
    angle_meas_new = -1 * to_signed(angle_meas_new, 17);
    update_sample(&angle_meas, angle_meas_new);
  }

  // enter controls on rising edge of ACC, exit controls on ACC off
  if (subaru_lkas_angle && (msg->addr == MSG_SUBARU_ES_Status) && (msg->bus == alt_main_bus)) {
    bool cruise_engaged = (msg->data[3] >> 5) & 1U;
    if (subaru_mads) {
      // latch on the rising edge of ACC and hold through an ACC dropout. the main switch
      // below is the only thing that exits controls.
      // cruise_engaged_prev is deliberately left tracking when the main switch is off, so
      // ACC held high across a main switch cycle cannot look like a fresh rising edge
      if (cruise_engaged && !cruise_engaged_prev && acc_main_on) {
        controls_allowed = true;
        controls_allowed_lateral = true;
      }
      cruise_engaged_prev = cruise_engaged;
    } else {
      pcm_cruise_check(cruise_engaged);
    }
  }

  // always exit controls on main switch off
  // Signal: ES_DashStatus.Cruise_On
  if (subaru_mads && (msg->addr == MSG_SUBARU_ES_DashStatus) && (msg->bus == SUBARU_CAM_BUS)) {
    const bool main_on = GET_BIT(msg, 49U);
    // with main switch arming on, switching cruise on is enough to enter controls. only on
    // the rising edge, never held: re-asserting every message would defeat the heartbeat
    // mismatch check that drops controls when openpilot stops reporting engaged.
    // subaru_main_on_prev starts true so the switch already being on at power up is not an
    // edge, which keeps this out of the window where openpilot is still initializing.
    if (subaru_mads_main && main_on && !subaru_main_on_prev) {
      controls_allowed = true;
      controls_allowed_lateral = true;
    }
    subaru_main_on_prev = main_on;

    acc_main_on = main_on;
    if (!acc_main_on) {
      controls_allowed = false;
      controls_allowed_lateral = false;
    }
  }
  if (!subaru_lkas_angle && (msg->addr == MSG_SUBARU_CruiseControl) && (msg->bus == alt_main_bus)) {
    bool cruise_engaged = (msg->data[5] >> 1) & 1U;
    pcm_cruise_check(cruise_engaged);
  }

  // update vehicle moving with any non-zero wheel speed
  if ((msg->addr == MSG_SUBARU_Wheel_Speeds) && (msg->bus == alt_main_bus)) {
    uint32_t fr = (GET_BYTES(msg, 1, 3) >> 4) & 0x1FFFU;
    uint32_t rr = (GET_BYTES(msg, 3, 3) >> 1) & 0x1FFFU;
    uint32_t rl = (GET_BYTES(msg, 4, 3) >> 6) & 0x1FFFU;
    uint32_t fl = (GET_BYTES(msg, 6, 2) >> 3) & 0x1FFFU;

    vehicle_moving = (fr > 0U) || (rr > 0U) || (rl > 0U) || (fl > 0U);

    UPDATE_VEHICLE_SPEED((fr + rr + rl + fl) / 4.0 * 0.057 * KPH_TO_MS);
  }

  if ((msg->addr == MSG_SUBARU_Brake_Status) && (msg->bus == alt_main_bus)) {
    brake_pressed = (msg->data[7] >> 6) & 1U;
  }

  if ((msg->addr == MSG_SUBARU_Throttle) && (msg->bus == SUBARU_MAIN_BUS)) {
    gas_pressed = msg->data[4] != 0U;
  }
}

static bool subaru_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits SUBARU_STEERING_LIMITS      = SUBARU_STEERING_LIMITS_GENERATOR(2047, 50, 70);
  const TorqueSteeringLimits SUBARU_GEN2_STEERING_LIMITS = SUBARU_STEERING_LIMITS_GENERATOR(1000, 40, 40);

  const LongitudinalLimits SUBARU_LONG_LIMITS = {
    .min_gas = 808,       // appears to be engine braking
    .max_gas = 3400,      // approx  2 m/s^2 when maxing cruise_rpm and cruise_throttle
    .inactive_gas = 1818, // this is zero acceleration
    .max_brake = 600,     // approx -3.5 m/s^2

    .min_transmission_rpm = 0,
    .max_transmission_rpm = 3600,
  };

  bool tx = true;
  bool violation = false;

  // angle steer cmd checks
  if (msg->addr == MSG_SUBARU_ES_LKAS_ANGLE) {
    // Based on SUBARU_ASCENT (most restrictive slip factor)
    const AngleSteeringLimits SUBARU_ANGLE_STEERING_LIMITS = {
      .max_angle = 190 * 100,
      .angle_deg_to_can = 100.,
      .frequency = 50U,
    };

    const AngleSteeringParams SUBARU_STEERING_PARAMS = {
      .slip_factor = -0.000580374471400815,
      .steer_ratio = 13.5,
      .wheelbase = 2.89,
    };

    int desired_angle = GET_BYTES(msg, 5, 3) & 0x1FFFFU;
    desired_angle = -1 * to_signed(desired_angle, 17);
    bool lkas_request = GET_BIT(msg, 12U);

    violation |= steer_angle_cmd_checks_vm(desired_angle, lkas_request, SUBARU_ANGLE_STEERING_LIMITS, SUBARU_STEERING_PARAMS);
  }

  // torque steer cmd checks
  if (msg->addr == MSG_SUBARU_ES_LKAS) {
    int desired_torque = ((GET_BYTES(msg, 0, 4) >> 16) & 0x1FFFU);
    desired_torque = -1 * to_signed(desired_torque, 13);

    bool steer_req = (msg->data[3] >> 5) & 1U;

    const TorqueSteeringLimits limits = subaru_gen2 ? SUBARU_GEN2_STEERING_LIMITS : SUBARU_STEERING_LIMITS;
    violation |= steer_torque_cmd_checks(desired_torque, steer_req, limits);
  }

  // check es_brake brake_pressure limits
  if (msg->addr == MSG_SUBARU_ES_Brake) {
    int es_brake_pressure = GET_BYTES(msg, 2, 2);
    violation |= longitudinal_brake_checks(es_brake_pressure, SUBARU_LONG_LIMITS);
  }

  // check es_distance cruise_throttle limits
  if (msg->addr == MSG_SUBARU_ES_Distance) {
    int cruise_throttle = (GET_BYTES(msg, 2, 2) & 0x1FFFU);
    bool cruise_cancel = (msg->data[7] >> 0) & 1U;

    if (subaru_longitudinal) {
      violation |= longitudinal_gas_checks(cruise_throttle, SUBARU_LONG_LIMITS);
    } else {
      // If openpilot is not controlling long, only allow ES_Distance for cruise cancel requests,
      // (when Cruise_Cancel is true, and Cruise_Throttle is inactive)
      violation |= (cruise_throttle != SUBARU_LONG_LIMITS.inactive_gas);
      violation |= (!cruise_cancel);
    }
  }

  // check es_status transmission_rpm limits
  if (msg->addr == MSG_SUBARU_ES_Status) {
    int transmission_rpm = (GET_BYTES(msg, 2, 2) & 0x1FFFU);
    violation |= longitudinal_transmission_rpm_checks(transmission_rpm, SUBARU_LONG_LIMITS);
  }

  // Auto Vehicle Hold. openpilot asks for this once while parked at the start of a drive,
  // so both gates below cost it nothing and make the message inert on the road.
  if (msg->addr == MSG_SUBARU_Comfort_Control) {
    // 1 is off, 2 is on, 0 is the idle frame. bytes 3 to 7 are the steady state the car
    // sends with the engine running, pinned here so openpilot can send this one frame and
    // no other. whatever else this message carries stays out of reach.
    //
    // Deliberately not gated on vehicle_moving. This button arms Auto Vehicle Hold, it does
    // not apply the brakes: the car only ever holds once it has already come to a stop under
    // the driver's own braking. So the request reaching a rolling car does exactly what the
    // driver's own thumb does on the touchscreen, and the content pin below is what keeps
    // this address from carrying anything else.
    const bool valid_request = (msg->data[2] == 0U) || (msg->data[2] == 1U) || (msg->data[2] == 2U);
    const bool valid_static = (msg->data[3] == 0x01U) && (msg->data[4] == 0x00U) && (msg->data[5] == 0x00U) &&
                              (msg->data[6] == 0x0EU) && (msg->data[7] == 0x00U);
    violation |= !subaru_avh;
    violation |= !valid_request;
    violation |= !valid_static;
  }

  // Auto start-stop engine shutoff. This one is the dash button rather than a state, so a
  // frame without the button bit set would just be openpilot competing with the car's own
  // copy of a message it has no business sending.
  //
  // Deliberately not gated on vehicle_moving, unlike Comfort_Control above. This button
  // touches nothing but the engine's own idle stop, so the worst it can do at speed is
  // change a setting that only takes effect at the next standstill. Openpilot is often not
  // running yet when the driver pulls away, and a start of drive gate the driver can beat
  // by leaving quickly is a gate that mostly just fails to help.
  if (msg->addr == MSG_SUBARU_Dashlights) {
    // Signal: Dashlights.STOP_START
    const bool stop_start_pressed = GET_BIT(msg, 54U);
    violation |= !subaru_stop_start;
    violation |= !stop_start_pressed;
  }

  if (msg->addr == MSG_SUBARU_ES_UDS_Request) {
    // tester present ('\x02\x3E\x80\x00\x00\x00\x00\x00') is allowed for gen2 longitudinal to keep eyesight disabled
    bool is_tester_present = (GET_BYTES(msg, 0, 4) == 0x00803E02U) && (GET_BYTES(msg, 4, 4) == 0x0U);

    // reading ES button data by identifier (b'\x03\x22\x11\x30\x00\x00\x00\x00') is also allowed (DID 0x1130)
    bool is_button_rdbi = (GET_BYTES(msg, 0, 4) == 0x30112203U) && (GET_BYTES(msg, 4, 4) == 0x0U);

    violation |= !(is_tester_present || is_button_rdbi);
  }

  if (violation){
    tx = false;
  }
  return tx;
}

static safety_config subaru_init(uint16_t param) {
  static const CanMsg SUBARU_TX_MSGS[] = {
    SUBARU_BASE_TX_MSGS(SUBARU_MAIN_BUS, MSG_SUBARU_ES_LKAS)
    SUBARU_COMMON_TX_MSGS(SUBARU_MAIN_BUS)
  };

  static const CanMsg SUBARU_LONG_TX_MSGS[] = {
    SUBARU_BASE_TX_MSGS(SUBARU_MAIN_BUS, MSG_SUBARU_ES_LKAS)
    SUBARU_COMMON_LONG_TX_MSGS(SUBARU_MAIN_BUS)
  };

  static const CanMsg SUBARU_GEN2_TX_MSGS[] = {
    SUBARU_BASE_TX_MSGS(SUBARU_ALT_BUS, MSG_SUBARU_ES_LKAS)
    SUBARU_COMMON_TX_MSGS(SUBARU_ALT_BUS)
  };

  static const CanMsg SUBARU_GEN2_LONG_TX_MSGS[] = {
    SUBARU_BASE_TX_MSGS(SUBARU_ALT_BUS, MSG_SUBARU_ES_LKAS)
    SUBARU_COMMON_LONG_TX_MSGS(SUBARU_ALT_BUS)
    SUBARU_GEN2_LONG_ADDITIONAL_TX_MSGS()
  };

  static const CanMsg SUBARU_LKAS_ANGLE_TX_MSGS[] = {
    SUBARU_BASE_TX_MSGS(SUBARU_MAIN_BUS, MSG_SUBARU_ES_LKAS_ANGLE)
    SUBARU_COMMON_TX_MSGS(SUBARU_MAIN_BUS)
    SUBARU_LKAS_ALERT_TX_MSGS()
  };

  static const CanMsg SUBARU_LKAS_ANGLE_GEN2_TX_MSGS[] = {
    SUBARU_BASE_TX_MSGS(SUBARU_ALT_BUS, MSG_SUBARU_ES_LKAS_ANGLE)
    SUBARU_COMMON_TX_MSGS(SUBARU_ALT_BUS)
    SUBARU_LKAS_ALERT_TX_MSGS()
    SUBARU_COMFORT_TX_MSGS(SUBARU_ALT_BUS)
  };

  static RxCheck subaru_rx_checks[] = {
    SUBARU_COMMON_RX_CHECKS(SUBARU_MAIN_BUS)
  };

  static RxCheck subaru_gen2_rx_checks[] = {
    SUBARU_COMMON_RX_CHECKS(SUBARU_ALT_BUS)
  };

  static RxCheck subaru_lkas_angle_rx_checks[] = {
    SUBARU_LKAS_ANGLE_RX_CHECKS(SUBARU_MAIN_BUS)
  };

  static RxCheck subaru_lkas_angle_gen2_rx_checks[] = {
    SUBARU_LKAS_ANGLE_RX_CHECKS(SUBARU_ALT_BUS)
  };

  static RxCheck subaru_lkas_angle_mads_rx_checks[] = {
    SUBARU_LKAS_ANGLE_MADS_RX_CHECKS(SUBARU_MAIN_BUS)
  };

  static RxCheck subaru_lkas_angle_gen2_mads_rx_checks[] = {
    SUBARU_LKAS_ANGLE_MADS_RX_CHECKS(SUBARU_ALT_BUS)
  };

  const uint16_t SUBARU_PARAM_GEN2 = 1;
  const uint16_t SUBARU_PARAM_LKAS_ANGLE = 8;
  const uint16_t SUBARU_PARAM_MADS = 16;
  const uint16_t SUBARU_PARAM_MADS_MAIN = 32;
  const uint16_t SUBARU_PARAM_AVH = 64;
  const uint16_t SUBARU_PARAM_STOP_START = 128;

  subaru_gen2 = GET_FLAG(param, SUBARU_PARAM_GEN2);
  subaru_lkas_angle = GET_FLAG(param, SUBARU_PARAM_LKAS_ANGLE);

#ifdef ALLOW_DEBUG
  const uint16_t SUBARU_PARAM_LONGITUDINAL = 2;
  subaru_longitudinal = GET_FLAG(param, SUBARU_PARAM_LONGITUDINAL);
#endif

  // MADS decouples steering from ACC, so it only applies to the angle cars we support it
  // on. openpilot longitudinal stays refused, but as a scope limit rather than a safety
  // necessity: a brake press drops controls_allowed and every longitudinal command with
  // it, and only controls_allowed_lateral survives, so the combination would be sound.
  subaru_mads = subaru_lkas_angle && !subaru_longitudinal && GET_FLAG(param, SUBARU_PARAM_MADS);

  // arming off the main switch alone is an extension of MADS, so it inherits those refusals
  subaru_mads_main = subaru_mads && GET_FLAG(param, SUBARU_PARAM_MADS_MAIN);

  // the comfort requests only exist on the alt bus of a gen2 angle car, which is the only
  // place the messages were measured. everywhere else they stay refused whatever is asked.
  const bool subaru_comfort_bus = subaru_lkas_angle && subaru_gen2;
  subaru_avh = subaru_comfort_bus && GET_FLAG(param, SUBARU_PARAM_AVH);
  subaru_stop_start = subaru_comfort_bus && GET_FLAG(param, SUBARU_PARAM_STOP_START);
  // assume the switch is already on, which it is with the car running
  subaru_main_on_prev = true;

  safety_config ret;
  if (subaru_lkas_angle && subaru_mads) {
    ret = subaru_gen2 ? BUILD_SAFETY_CFG(subaru_lkas_angle_gen2_mads_rx_checks, SUBARU_LKAS_ANGLE_GEN2_TX_MSGS) : \
                        BUILD_SAFETY_CFG(subaru_lkas_angle_mads_rx_checks, SUBARU_LKAS_ANGLE_TX_MSGS);
  } else if (subaru_lkas_angle) {
    ret = subaru_gen2 ? BUILD_SAFETY_CFG(subaru_lkas_angle_gen2_rx_checks, SUBARU_LKAS_ANGLE_GEN2_TX_MSGS) : \
                        BUILD_SAFETY_CFG(subaru_lkas_angle_rx_checks, SUBARU_LKAS_ANGLE_TX_MSGS);
  } else if (subaru_gen2) {
    ret = subaru_longitudinal ? BUILD_SAFETY_CFG(subaru_gen2_rx_checks, SUBARU_GEN2_LONG_TX_MSGS) : \
                                BUILD_SAFETY_CFG(subaru_gen2_rx_checks, SUBARU_GEN2_TX_MSGS);
  } else {
    ret = subaru_longitudinal ? BUILD_SAFETY_CFG(subaru_rx_checks, SUBARU_LONG_TX_MSGS) : \
                                BUILD_SAFETY_CFG(subaru_rx_checks, SUBARU_TX_MSGS);
  }
  return ret;
}

const safety_hooks subaru_hooks = {
  .init = subaru_init,
  .rx = subaru_rx_hook,
  .tx = subaru_tx_hook,
  .get_counter = subaru_get_counter,
  .get_checksum = subaru_get_checksum,
  .compute_checksum = subaru_compute_checksum,
};
