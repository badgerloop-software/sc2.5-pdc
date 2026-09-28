#include "canPDC.h"
#include "const.h"

volatile bool forwardAndReverse = false;

static uint8_t lastDriveMode = 0xFF;

static constexpr uint16_t THROTTLE_SENT_MAX = 4095U;

static inline uint16_t readBeUint16(const uint8_t *data) {
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

// Battery SOC constants
static const float CELL_VOLTAGE_SCALE_V = 0.0001f;

// OCV curve: cell V at SOC 0,5,10,...,100 (matches streamlit app)
static const float CELL_VOLTAGES[] = {
    2.500f, 2.871f, 3.043f, 3.160f, 3.269f,
    3.371f, 3.429f, 3.476f, 3.521f, 3.572f,
    3.629f, 3.680f, 3.726f, 3.765f, 3.804f,
    3.865f, 3.923f, 3.955f, 3.975f, 3.998f, 4.111f
};
static const int CELL_VOLTAGE_COUNT = sizeof(CELL_VOLTAGES) / sizeof(CELL_VOLTAGES[0]);
static const float SERIES_COUNT = 29.0f;

static float socFromPackVoltage(float pack_voltage) {
    float cell_voltage = pack_voltage / SERIES_COUNT;

    if (cell_voltage <= CELL_VOLTAGES[0]) {
        return 0.0f;
    }
    if (cell_voltage >= CELL_VOLTAGES[CELL_VOLTAGE_COUNT - 1]) {
        return 100.0f;
    }

    for (int i = 0; i < CELL_VOLTAGE_COUNT - 1; i++) {
        float low_v = CELL_VOLTAGES[i];
        float high_v = CELL_VOLTAGES[i + 1];
        if (low_v <= cell_voltage && cell_voltage <= high_v) {
            float fraction = (cell_voltage - low_v) / (high_v - low_v);
            return (float)(i * 5) + fraction * 5.0f;
        }
    }

    return 0.0f;
}

CANPDC::CANPDC(CAN_TypeDef *canPort, CAN_PINS pins, int frequency)
    : CANManager(canPort, pins, frequency) {};

void CANPDC::readHandler(CAN_message_t msg) {

  float local_high_cell_v = 0.0f;
  float local_low_cell_v = 0.0f;
  float local_est_pack_v = 0.0f;
  
  switch (msg.id) {
  case 0x109:{
            // High/Low cell V (0.0001 V), Pack Abs Current (0x8000 midscale, 0.1 A)
            local_high_cell_v = (float)readBeUint16(&msg.buf[0]) * CELL_VOLTAGE_SCALE_V;
            local_low_cell_v = (float)readBeUint16(&msg.buf[2]) * CELL_VOLTAGE_SCALE_V;
            local_est_pack_v = ((local_high_cell_v + local_low_cell_v) / 2.0f) * SERIES_COUNT;
            battery_soc = socFromPackVoltage(local_est_pack_v);
            break;
  }
  case FORWARD_AND_REVERSE_ID: { // 0x300
    // Byte 0 bit layout per CAN spec:
    //   bit 0: headlight, bit 1: left_blink, bit 2: right_blink,
    //   bit 3: direction_switch, bit 4: horn
    // bit 3 direction_switch: 1 = forward, 0 = reverse (steering wheel)
    bool forward_selected = ((msg.buf[0] >> 3) & 1) != 0;
    forwardAndReverse = forward_selected ? FORWARD_VALUE : REVERSE_VALUE;
    break;
  }

  case REGEN_BRAKE_INPUT_ID: { // 0x301 regen brake from steering wheel (normalized 0.0–1.0)
    if (msg.len >= sizeof(float)) {
      float regen_val = 0.0f;
      memcpy((void *)&regen_val, msg.buf, sizeof(float));
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
  this->sendMessage(0x206, (void *)&brake_pressure_telem,
                    sizeof(float)); // Brake pressure in PSI
  this->sendMessage(0x207, (void *)&digital_data, sizeof(digital_data));
  this->sendMessage(0x208, (void *)&mph, sizeof(float));
  // 0x209/0x302 are INPUTs received from the pedal/test board — do not
  // re-broadcast.
}
