#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include "Arduino.h"
#include "const.h"
#include "IOManagement.h"
#include "STM32TimerInterrupt_Generic.h"
#include "canPDC.h"

void initPDCState();
void transition();
PDCStates get_state();

extern volatile float cruise_target_mph;
extern volatile bool cruise_inc_pending;
extern volatile bool cruise_dec_pending;

#endif
