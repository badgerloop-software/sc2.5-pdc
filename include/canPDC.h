#ifndef __CAN_PDC_H__
#define __CAN_PDC_H__

#include "IOManagement.h"
#include "canmanager.h"

// Macros for the CAN message IDs
#define FORWARD_AND_REVERSE_ID 0x300
#define REGEN_BRAKE_INPUT_ID 0x301
#define THROTTLE_INPUT_ID 0x302
#define DRIVE_MODE_INPUT_ID 0x303
// 0x210 payload: byte 0 = active (0/1), bytes 1-4 = target MPH (float).
#define CRUISE_STATUS_OUTPUT_ID 0x210

class CANPDC : public CANManager {
public:
  CANPDC(CAN_TypeDef *canPort, CAN_PINS pins, int frequency = DEFAULT_CAN_FREQ);
  void readHandler(CAN_message_t msg);
  // Sends the cruise active state and target MPH when either changes.
  void sendPDCData();
};

extern volatile bool forwardAndReverse;
extern volatile bool cruise_main;
extern volatile bool cruise_inc;
extern volatile bool cruise_dec;

// 0x300 requires two bytes: existing controls, then regen button in bit 0
extern volatile bool regen_button_pressed;
extern volatile bool cruise_rearm_required;
// Call only from the state timer or with interrupts masked
void resetCruiseButtonEvents();

#endif
