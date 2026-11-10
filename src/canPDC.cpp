#include "canPDC.h"
#include "const.h"
#include <zephyr/kernel.h>
#include <zephyr/devicetree/gpio.h>

volatile bool forwardAndReverse = false;

static uint8_t lastDriveMode = 0xFF;

static constexpr uint16_t THROTTLE_SENT_MAX = 4095U;

CANPDC::CANPDC(const struct device * canDevice, uint32_t frequency)
    : ZephyrCAN(canDevice, arr_ids, sizeof(ARR_IDS) / sizeof(ARR_IDS[0]), frequency) {};


void CANPDC::readHandler(struct can_frame * msg) {
  switch (msg->id) {
  case FORWARD_AND_REVERSE_ID: { // 0x300
    // Byte 0 bit layout per CAN spec:
    //   bit 0: headlight, bit 1: left_blink, bit 2: right_blink,
    //   bit 3: direction_switch, bit 4: horn
    // bit 3 direction_switch: 1 = forward, 0 = reverse (steering wheel)
    bool forward_selected = ((msg->data[0] >> 3) & 1) != 0;
    forwardAndReverse = forward_selected ? FORWARD_VALUE : REVERSE_VALUE;
    break;
  }

  case REGEN_BRAKE_INPUT_ID: { // 0x301 regen brake from steering wheel (normalized 0.0–1.0)
    if (msg->can_dlc >= sizeof(float)) {
      float regen_val = 0.0f;
      memcpy((void *)&regen_val, msg->data, sizeof(float));
      if (regen_val < 0.0f)
        regen_val = 0.0f;
      else if (regen_val > 1.0f)
        regen_val = 1.0f;
      regen_in = regen_val;
    }
    break;
  }

#ifndef TEST_MODE
  case THROTTLE_INPUT_ID: { // 0x302 uint16 throttle command in production.
    uint16_t throttle_raw = 0;
    memcpy((void *)&throttle_raw, msg->data, sizeof(uint16_t));
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
    if (msg->can_dlc >= 1 && msg->data[0] != lastDriveMode) {
      lastDriveMode = msg->data[0];
      set_eco_mode(lastDriveMode == 0);
    }
    break;
  }
#endif

#ifdef TEST_MODE
  case 0x209: // acc_in — sent by test board simulating the pedal.
              // Only used in TEST_MODE; in production 0x302 is used.
    memcpy((void *)&acc_in, msg.buf, sizeof(float));
    break;
#endif

  default:
    break;
  }
}

void CANPDC::sendPDCData() {
  this->sendMessage(0x200, (void *)&acc_out, sizeof(float));
  this->sendMessage(0x201, (void *)&regen_brake, sizeof(float));
  this->sendMessage(0x202, (void *)&lv_12V_telem, sizeof(float));
  this->sendMessage(0x203, (void *)&lv_5V_telem, sizeof(float));
  this->sendMessage(0x204, (void *)&lv_5V_current, sizeof(float));
  this->sendMessage(0x205, (void *)&current_in_telem, sizeof(float));
  this->sendMessage(0x206, (void *)&brake_pressure_telem, sizeof(float));
  this->sendMessage(0x207, (void *)&digital_data, sizeof(digital_data));
  this->sendMessage(0x208, (void *)&mph, sizeof(float));
  // 0x209/0x302 are INPUTs received from the pedal/test board — do not
  // re-broadcast.
}