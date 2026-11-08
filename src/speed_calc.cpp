#include "speed_calc.h"
#include "IOManagement.h"


define MOTOR_PIN_INTERUPT DT_PATH(zephyr_user)
static const struct gpio_dt_spec intr_pin = GPIO_DT_SPEC_GET(MOTOR_PIN_INTERUPT, interrupt_gpios);

volatile uint8_t speedPulses = 0;
static volatile uint8_t previousPulses[ARRAY_SIZE];

static struct gpio_callback mcu_speed_sig_cb; 


K_TIMER_DEFINE(speedCalcTimer, calculateSpeed, NULL);

void increment() {
    speedPulses++;
}

void calculateSpeed(){
    // RPM is always derived from physical MC_SPEED_SIG pulses so the state
    // machine tracks the real motor speed, even when TEST_MODE is active.

    // Variable to track array update position
    static uint8_t calculationCounter = 0;
    static uint16_t runningSum = 0; 

    // Replaces oldest pulse in pulse counter array with a new one
    calculationCounter = (calculationCounter + 1) % ARRAY_SIZE;
    runningSum = runningSum - previousPulses[calculationCounter] + speedPulses;
    previousPulses[calculationCounter] = speedPulses;
    speedPulses = 0;
}

// do not use in ISR context, as it uses floating point 
void calculateMph(uint16_t runningSum, float &rpm, float &mph) {
   float revolutions = runningSum / (float)PULSES_PER_REV;
    rpm = revolutions / (ARRAY_SIZE * SPEED_CALC_INTERVAL_US / 1000000.0 / 60.0 );
    mph = rpm * WHEEL_CIRCUMFERENCE * 0.00094696;

}

void startSpeedCalculation() {
    gpio_pin_configure_dt(&intr_pin, GPIO_INPUT);
    gpio_init_callback(&mcu_speed_sig_cb, intr_handler, BIT(intr_pin.pin));
    gpio_add_callback(&mcu_speed_sig_cb, increment, BIT(intr_pin.pin));
    gpio_pin_interrupt_configure_dt(&intr_pin, GPIO_INT_EDGE_RISING);
   k_timer_start(&speedCalcTimer, K_USEC(SPEED_CALC_INTERVAL_US), K_USEC(SPEED_CALC_INTERVAL_US));
}
