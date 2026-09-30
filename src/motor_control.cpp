
#include "motor_control.h"
#include "IOManagement.h"
#include "speed_calc.h"
#include "canPDC.h"

volatile PDCStates pdcState = PDCStates::OFF;
volatile bool cruise_active = false;
volatile float cruise_target_mph = 0.0f;
volatile bool cruise_inc_pending = false;
volatile bool cruise_dec_pending = false;

static bool previous_cruise_inc = false;
static bool previous_cruise_dec = false;

STM32TimerInterrupt state_updater(TIM2);

void initPDCState() {
  pdcState = PDCStates::PARK;
  state_updater.attachInterruptInterval(IO_UPDATE_PERIOD, transition);
  set_eco_mode(true);
  set_direction(FORWARD_VALUE);
}

PDCStates get_state() { return pdcState; }

void transition() {
  const bool cruise_inc_rising = cruise_inc && !previous_cruise_inc;
  const bool cruise_dec_rising = cruise_dec && !previous_cruise_dec;
  previous_cruise_inc = cruise_inc;
  previous_cruise_dec = cruise_dec;

  switch (pdcState) {
  case PDCStates::PARK:
    if (!digital_data.park_brake) {
      pdcState = PDCStates::IDLE;
    }
    writeAccOut(0.0);
    writeRegenBrake(0.0);
    break;

  case PDCStates::IDLE:
    if (digital_data.park_brake) {
      pdcState = PDCStates::PARK;
      break;
    }

    set_direction(forwardAndReverse);
    writeAccOut(acc_in);
    writeRegenBrake(regen_in);

    if (rpm >= MIN_MOVING_SPEED) {
      if (forwardAndReverse == FORWARD_VALUE) {
        pdcState = PDCStates::FORWARD;
      } else {
        pdcState = PDCStates::REVERSE;
      }
    }
    break;

  case PDCStates::REVERSE:
    if (rpm < MIN_MOVING_SPEED) {
      pdcState = PDCStates::IDLE;
    }
    set_direction(REVERSE_VALUE);
    writeAccOut(acc_in);
    writeRegenBrake(regen_in);
    break;

  case PDCStates::FORWARD:
    if (rpm < MIN_MOVING_SPEED) {
      pdcState = PDCStates::IDLE;
      break;
    }

    set_direction(FORWARD_VALUE);
    writeAccOut(acc_in);
    writeRegenBrake(regen_in);
    break;

  default:
    pdcState = PDCStates::PARK;
    break;
  }

  if (pdcState == PDCStates::FORWARD) {
    cruise_inc_pending = cruise_inc_pending || cruise_inc_rising;
    cruise_dec_pending = cruise_dec_pending || cruise_dec_rising;
  } else {
    cruise_inc_pending = false;
    cruise_dec_pending = false;
  }

  if (digital_data.park_brake) {
    writeAccOut(0.0);
    writeRegenBrake(0.0);
  }

  if (brake_pressed) {
    writeAccOut(0.0);
    writeRegenBrake(0.0);
  }
}
