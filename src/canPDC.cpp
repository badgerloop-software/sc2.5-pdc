#include "canPDC.h"
#include "const.h"
#include "motor_control.h"
#include <math.h>

volatile bool forwardAndReverse = false;
volatile bool cruise_main = false;
volatile bool cruise_inc = false;
volatile bool cruise_dec = false;

volatile bool regen_button_pressed = false;
volatile bool cruise_rearm_required = true;
static bool increase_button_armed = false;
static bool decrease_button_armed = false;

void resetCruiseButtonEvents() {
  cruise_inc_pending = false;
  cruise_dec_pending = false;
  increase_button_armed = false;
  decrease_button_armed = false;
}

// Preserve the interrupt mask so reception and timer consumption cannot interleave
class SteeringReceiveGuard {
 public:
  SteeringReceiveGuard() : previous_interrupt_mask(__get_PRIMASK()) { __disable_irq(); }
  ~SteeringReceiveGuard() { __set_PRIMASK(previous_interrupt_mask); }
 private:
  uint32_t previous_interrupt_mask;
};

static uint8_t lastDriveMode = 0xFF;

static constexpr uint16_t THROTTLE_SENT_MAX = 4095U;

CANPDC::CANPDC(CAN_TypeDef *canPort, CAN_PINS pins, int frequency)
    : CANManager(canPort, pins, frequency) {};

void CANPDC::readHandler(CAN_message_t msg) {
  SteeringReceiveGuard receive_guard;
  switch (msg.id) {
  case FORWARD_AND_REVERSE_ID: { // 0x300
    if (msg.len < 2) break;
    // Byte 0 bit layout per CAN spec:
    //   bit 0: headlight, bit 1: left_blink, bit 2: right_blink,
    //   bit 3: direction_switch, bit 4: horn, bit 5: cruise main,
    //   bit 6: cruise increase, bit 7: cruise decrease
    // Bit 3 direction_switch: 0 = forward, 1 = reverse on both boards
    bool reverse_selected = ((msg.buf[0] >> 3) & 1) != 0;
    forwardAndReverse = reverse_selected ? REVERSE_VALUE : FORWARD_VALUE;
    cruise_main = ((msg.buf[0] >> 5) & 1) != 0;
    cruise_inc = ((msg.buf[0] >> 6) & 1) != 0;
    cruise_dec = ((msg.buf[0] >> 7) & 1) != 0;
    regen_button_pressed = (msg.buf[1] & 1U) != 0;
    if (!cruise_main) {
      cruise_rearm_required = false;
    }
    if (pdcState == PDCStates::CRUISE && regen_button_pressed) {
      cruise_rearm_required = true;
    }
    if (cruise_inc && cruise_dec) {
      resetCruiseButtonEvents();
      break;
    }
    const bool accept_adjustment = pdcState == PDCStates::CRUISE &&
        cruise_main && !cruise_rearm_required && !regen_button_pressed;
    if (cruise_inc && increase_button_armed && accept_adjustment) cruise_inc_pending = true;
    if (cruise_dec && decrease_button_armed && accept_adjustment) cruise_dec_pending = true;
    increase_button_armed = !cruise_inc;
    decrease_button_armed = !cruise_dec;
    if (!accept_adjustment) {
      cruise_inc_pending = false;
      cruise_dec_pending = false;
    }
    break;
  }

  case REGEN_BRAKE_INPUT_ID: { // 0x301 regen brake from steering wheel (normalized 0.0–1.0)
    if (msg.len >= sizeof(float)) {
      float regen_val = 0.0f;
      memcpy((void *)&regen_val, msg.buf, sizeof(float));
      if (!isfinite(regen_val)) break;
      if (regen_val < 0.0f)
        regen_val = 0.0f;
      else if (regen_val > 1.0f)
        regen_val = 1.0f;
      regen_in = regen_val;
      if (pdcState == PDCStates::CRUISE && regen_val > 0.01f) {
        cruise_rearm_required = true;
      }
    }
    break;
  }

#ifndef TEST_MODE
  case THROTTLE_INPUT_ID: { // 0x302 uint16 throttle command in production.
    if (msg.len < sizeof(uint16_t)) break;
    uint16_t throttle_raw = 0;
    memcpy((void *)&throttle_raw, msg.buf, sizeof(uint16_t));
    acc_in_raw = throttle_raw;

    // Steering wheel sends calibrated counts in [0, THROTTLE_SENT_MAX].
    float normalized = (float)throttle_raw / (float)THROTTLE_SENT_MAX;
    if (normalized < 0.0f) {
      normalized = 0.0f;
    } else if (normalized > 1.0f) {
      normalized = 1.0f;
    }
    acc_in = normalized;
    break;
  }

  case DRIVE_MODE_INPUT_ID: { // 0x303 eco/pwr from steering wheel (0=Eco, 1=Pwr)
    if (msg.len >= 1 && msg.buf[0] != lastDriveMode) {
      lastDriveMode = msg.buf[0];
      set_eco_mode(lastDriveMode == 0);
    }
    break;
  }
#endif

#ifdef TEST_MODE
  case 0x209: { // acc_in — sent by test board simulating the pedal.
              // Only used in TEST_MODE; in production 0x302 is used.
    if (msg.len < sizeof(float)) break;
    float test_throttle;
    memcpy(&test_throttle, msg.buf, sizeof(float));
    if (!isfinite(test_throttle)) break;
    acc_in = test_throttle;
    break;
  }
#endif

  default:
    break;
  }
}

void CANPDC::sendPDCData() {
  static uint32_t last_cruise_status_update_version = 0;

  this->sendMessage(0x200, (void *)&acc_out, sizeof(float));
  this->sendMessage(0x201, (void *)&regen_brake, sizeof(float));
  this->sendMessage(0x202, (void *)&lv_12V_telem, sizeof(float));
  this->sendMessage(0x203, (void *)&lv_5V_telem, sizeof(float));
  this->sendMessage(0x204, (void *)&lv_5V_current, sizeof(float));
  this->sendMessage(0x205, (void *)&current_in_telem, sizeof(float));
  this->sendMessage(0x206, (void *)&brake_pressure_telem, sizeof(float));
  this->sendMessage(0x207, (void *)&digital_data, sizeof(digital_data));
  this->sendMessage(0x208, (void *)&mph, sizeof(float));

  // The state-machine timer records cruise state and target changes; transmit
  // from the main loop so CAN I/O is never performed from the timer interrupt.
  if (cruise_status_update_version != last_cruise_status_update_version) {
    uint32_t cruise_status_version;
    uint8_t cruise_status[1 + sizeof(float)];

    // Snapshot all values together; the timer may update them asynchronously.
    noInterrupts();
    cruise_status_version = cruise_status_update_version;
    cruise_status[0] = pdcState == PDCStates::CRUISE ? 1U : 0U;
    const float cruise_target = cruise_target_mph;
    memcpy(&cruise_status[1], &cruise_target, sizeof(cruise_target));
    interrupts();

    if (this->sendMessage(CRUISE_STATUS_OUTPUT_ID, cruise_status,
                          sizeof(cruise_status))) {
      last_cruise_status_update_version = cruise_status_version;
    }
  }
  // 0x209/0x302 are INPUTs received from the pedal/test board — do not
  // re-broadcast.
}
