
#include "motor_control.h"
#include "IOManagement.h"
#include "speed_calc.h"
#include "canPDC.h"

volatile PDCStates pdcState = PDCStates::OFF;
volatile float cruise_target_mph = 0.0f;
volatile uint32_t cruise_status_update_version = 0;
volatile bool cruise_inc_pending = false;
volatile bool cruise_dec_pending = false;

static constexpr float MAX_CRUISE_MPH = 45.0f;
static constexpr float MIN_CRUISE_SPEED_MPH = 3.0f;
static constexpr float CRUISE_DISENGAGE_INPUT_THRESHOLD = 0.01f;
static PID cruise_pid(SPEED_P_PARAM, SPEED_I_PARAM, SPEED_D_PARAM,
            PID_UPDATE_INTERVAL);

STM32TimerInterrupt state_updater(TIM2);

static float clampCruiseTarget(float target_mph) {
  if (target_mph < MIN_CRUISE_SPEED_MPH) {
    return MIN_CRUISE_SPEED_MPH;
  }
  if (target_mph > MAX_CRUISE_MPH) {
    return MAX_CRUISE_MPH;
  }
  return target_mph;
}


void initPDCState() {
  pdcState = PDCStates::PARK;
  cruise_pid.setInputLimits(0.0f, MAX_CRUISE_MPH);
  cruise_pid.setOutputLimits(MIN_OUT, MAX_OUT);
  cruise_pid.setMode(MANUAL_MODE);
  state_updater.attachInterruptInterval(IO_UPDATE_PERIOD, transition);
  set_eco_mode(true);
  set_direction(FORWARD_VALUE);
}

PDCStates get_state() { return pdcState; }

void transition() {
  const PDCStates state_before_transition = pdcState;
  const float cruise_target_before_transition = cruise_target_mph;
  if (pdcState == PDCStates::CRUISE &&
      (brake_pressed || regen_button_pressed || regen_in > CRUISE_DISENGAGE_INPUT_THRESHOLD)) {
    cruise_rearm_required = true;
  }

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

    if (cruise_main && !cruise_rearm_required && !brake_pressed &&
        !regen_button_pressed && forwardAndReverse == FORWARD_VALUE && mph >= MIN_CRUISE_SPEED_MPH &&
        regen_in <= CRUISE_DISENGAGE_INPUT_THRESHOLD) {
      cruise_target_mph = clampCruiseTarget(mph);
      cruise_pid.setSetPoint(cruise_target_mph);
      cruise_pid.setMode(AUTO_MODE);
      pdcState = PDCStates::CRUISE;
    }
    break;

  case PDCStates::CRUISE:
    if (mph < MIN_CRUISE_SPEED_MPH) {
      cruise_pid.setMode(MANUAL_MODE);
      pdcState = (rpm < MIN_MOVING_SPEED) ? PDCStates::IDLE : PDCStates::FORWARD;
      writeAccOut(0.0);
      writeRegenBrake(0.0);
      break;
    }
    if (!cruise_main || cruise_rearm_required || brake_pressed || regen_button_pressed ||
      regen_in > CRUISE_DISENGAGE_INPUT_THRESHOLD ||
      forwardAndReverse != FORWARD_VALUE) {
      cruise_pid.setMode(MANUAL_MODE);
      cruise_inc_pending = false;
      cruise_dec_pending = false;
      pdcState = PDCStates::FORWARD;
      writeAccOut(acc_in);
      writeRegenBrake(regen_in);
      break;
    }

    set_direction(FORWARD_VALUE);

    if (cruise_inc_pending) {
      cruise_target_mph += 1.0f;
      cruise_inc_pending = false;
    }
    if (cruise_dec_pending) {
      cruise_target_mph -= 1.0f;
      cruise_dec_pending = false;
    }

    cruise_target_mph = clampCruiseTarget(cruise_target_mph);
    cruise_pid.setSetPoint(cruise_target_mph);
    cruise_pid.setProcessValue(mph);
    writeAccOut(cruise_pid.compute());
    writeRegenBrake(regen_in);
    break;

  default:
    pdcState = PDCStates::PARK;
    break;
  }

  const bool cruise_active_state_changed =
      (state_before_transition == PDCStates::CRUISE) !=
      (pdcState == PDCStates::CRUISE);
  if (cruise_active_state_changed ||
      cruise_target_mph != cruise_target_before_transition) {
    ++cruise_status_update_version;
  }

  if (pdcState != PDCStates::CRUISE) {
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
