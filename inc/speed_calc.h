#ifndef __SPEED_CALC_H__
#define __SPEED_CALC_H__

#include "const.h"
#include "IOManagement.h"

#define SPEED_CALC_INTERVAL_US 50000 // us
#define PULSES_ARRAY_SIZE 20 
#define PULSES_PER_REV 48
#define WHEEL_CIRCUMFERENCE 69 // inches

#define MCU_SPEED_SIG PA8

void startSpeedCalculation();


// Starts speed (RPM and MPH) calculations at specified interval
extern volatile float mph;
extern volatile uint8_t previousPulses[PULSES_ARRAY_SIZE];

#endif  