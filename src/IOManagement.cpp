#include "IOManagement.h"
#include <zephyr/kernel.h>
#include <zephyr/devicetree/gpio.h>
#include <zephyr/logging/log.h> 

#define MOTOR_SPEED_NODE DT_PATH(motor_speed)
#define MCU_DIR_NODE     DT_PATH(mcu_dir)
#define MCU_ECO_NODE     DT_PATH(mcu_eco)
#define MCU_MC_ON_NODE   DT_PATH(mcu_mc_on)
#define PRK_TELEM_NODE   DT_PATH(prk_telem)
#define BRK_TELEM_NODE   DT_PATH(brk_telem)

volatile Digital_Data digital_data;
LOG_MODULE_REGISTER(my_app_module, LOG_LEVEL_DBG);
 // Callback for speed signal interrupt
static struct gpio_callback mcu_speed_sig_cb; 

static const struct gpio_dt_spec motor_speed =
    GPIO_DT_SPEC_GET(MOTOR_SPEED_NODE, interrupt_gpios);

static const struct gpio_dt_spec mcu_dir =
    GPIO_DT_SPEC_GET(MCU_DIR_NODE, interrupt_gpios);

static const struct gpio_dt_spec mcu_eco =
    GPIO_DT_SPEC_GET(MCU_ECO_NODE, interrupt_gpios);

static const struct gpio_dt_spec mcu_mc_on =
    GPIO_DT_SPEC_GET(MCU_MC_ON_NODE, interrupt_gpios);

static const struct gpio_dt_spec prk_telem =
    GPIO_DT_SPEC_GET(PRK_TELEM_NODE, interrupt_gpios);

static const struct gpio_dt_spec brk_telem =
    GPIO_DT_SPEC_GET(BRK_TELEM_NODE, interrupt_gpios);
K_TIMER_DEFINE(IOTimer, calculateSpeed, NULL);


volatile uint16_t acc_in_raw = 0;
volatile float acc_in = 0;
volatile float acc_out = 0;
volatile float regen_in = 0;
volatile float regen_brake = 0;
volatile float lv_12V_telem = 0;
volatile float lv_5V_telem = 0;
volatile float lv_5V_current = 0;
volatile float current_in_telem = 0;
volatile bool brake_pressed = false;
volatile float brake_pressure_telem = 0.0f;
volatile float mph = 0;
volatile float rpm = 0;

#ifdef DEBUG_PRINTS
static void agentDebugLogBrakeDigital(uint32_t dig, uint32_t mode, uint32_t pupd,
                                      uint32_t idr) {
  // #region agent log


  LOG_DBG(
      "{\"sessionId\":\"03e5a9\",\"runId\":\"digital-pd\",\"hypothesisId\":\"H\","
      "\"location\":\"IOManagement.cpp:readIO\",\"message\":\"brake_digital\","
      "\"data\":{\"brakeDigital\":%lu,\"pa0GpioMode\":%lu,\"pa0Pupd\":%lu,"
      "\"pa0Idr\":%lu,\"brakePressed\":%u,\"brakeTelem\":%.3f},"
      "\"timestamp\":%lu}\n",
      dig, mode, pupd, idr, (unsigned)brake_pressed, brake_pressure_telem,
      k_uptime_get());

  // Serial.printf(
  //     "{\"sessionId\":\"03e5a9\",\"runId\":\"digital-pd\",\"hypothesisId\":\"H\","
  //     "\"location\":\"IOManagement.cpp:readIO\",\"message\":\"brake_digital\","
  //     "\"data\":{\"brakeDigital\":%lu,\"pa0GpioMode\":%lu,\"pa0Pupd\":%lu,"
  //     "\"pa0Idr\":%lu,\"brakePressed\":%u,\"brakeTelem\":%.3f},"
  //     "\"timestamp\":%lu}\n",
  //   dig, mode, pupd, idr, (unsigned)brake_pressed, brake_pressure_telem,
  //    millis());
  // #endregion
}
#endif

// Ticker to poll input readings at fixed rate
//STM32TimerInterrupt IOTimer(TIM7);




// RPM pulse counting
// MC_SPEED_SIG fires 48 pulses per revolution.
// readIO() runs every IO_UPDATE_PERIOD µs and computes RPM from the count.
static volatile uint32_t pulseCount = 0;
#define PULSES_PER_REV 48

// Sample-and-delay debounce state for BRAKE_TELEM (updated in readIO).
static bool brake_last_raw = false;
static uint8_t brake_stable_count = 0;

static void speedPulseISR() { pulseCount++; }

void initSpeedCounter() {
  // attachInterrupt(digitalPinToInterrupt(MCU_SPEED_SIG), speedPulseISR, RISING);
  gpio_pin_configure_dt(&mcu_speed, GPIO_INPUT);
  gpio_init_callback(&mcu_speed_sig_cb, speedPulseISR, BIT(mcu_speed.pin));
  gpio_add_callback(&mcu_speed, BIT(mcu_speed.pin));
  gpio_pin_interrupt_configure_dt(&mcu_speed, GPIO_INT_EDGE_RISING);

} 

DAC_HandleTypeDef hdac;

void initDAC() {
    __HAL_RCC_DAC1_CLK_ENABLE();
    
    hdac.Instance = DAC1;
    HAL_DAC_Init(&hdac);

    DAC_ChannelConfTypeDef config = {0};
    config.DAC_Trigger = DAC_TRIGGER_NONE;
    config.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    HAL_DAC_ConfigChannel(&hdac, &config, DAC_CHANNEL_1);  // PA4 - regen
    HAL_DAC_ConfigChannel(&hdac, &config, DAC_CHANNEL_2);  // PA5 - accel
    
    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
    HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

    // PA5 is the default SPI SCK and may be claimed by the Arduino core.
    // Explicitly set it to analog mode so the DAC output isn't overridden.
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin  = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void initIO() {

  gpio_pin_configure_dt(&mcu_dir, GPIO_OUTPUT);
  gpio_pin_set_dt(&mcu_dir, FORWARD_VALUE);

  gpio_pin_configure_dt(&mcu_eco, GPIO_OUTPUT);
  gpio_pin_configure_dt(&mcu_mc_on, GPIO_INPUT);
  gpio_pin_configure_dt(&motor_speed, GPIO_INPUT);
  gpio_pin_configure_dt(&prk_telem, GPIO_INPUT);
  gpio_pin_configure_dt(&brk_telem, GPIO_INPUT | GPIO_PULL_DOWN);

  initDAC();
  initADC(ADC1);
  initSpeedCounter();


  k_timer_start(&IOTimer, K_USEC(IO_UPDATE_PERIOD), K_USEC(IO_UPDATE_PERIOD));

  if (k_timer_remaining_get(&IOTimer) > 0) {
#ifdef DEBUG_PRINTS
    printf("starting IO timer\n");
#endif
  } else {
#ifdef DEBUG_PRINTS
    printf("problem starting IO timer\n");
#endif
  }
}

void readIO() {
  // Compute RPM from pulse count since last call.
  // IO_UPDATE_PERIOD is in µs, convert to minutes for RPM.
  uint32_t count = pulseCount;
  pulseCount = 0;
  float interval_min = IO_UPDATE_PERIOD / 60000000.0f; // µs to minutes
  // rpm = (float)count / PULSES_PER_REV / interval_min;

  digital_data.mc_speed_sig = digitalRead(MCU_SPEED_SIG);

#ifndef TEST_MODE
  digital_data.park_brake = false; // digitalRead(PRK_BRK_TELEM);
#endif

  // acc_in is sourced from CAN in readHandler (0x302 production, 0x209 test).
  lv_12V_telem = readADC(ADC_CHANNEL_6) * 3.3 * 35.1 / 5.1;              // PA_1
  lv_5V_telem = readADC(ADC_CHANNEL_12) * 3.3 * 15.1 / 5.1;              // PA_7
  lv_5V_current = readADC(ADC_CHANNEL_15) * INA180_CURRENT_MULTIPLIER;   // PB_0
  current_in_telem = readADC(ADC_CHANNEL_8) * INA180_CURRENT_MULTIPLIER; // PA_3
#ifndef TEST_MODE
  // Digital brake switch: HIGH = pressed. Internal pulldown holds idle at 0.
  // Sample-and-delay: require N stable samples before updating brake_pressed.
  {
    bool raw = digitalRead(BRAKE_TELEM) == HIGH;
    if (raw != brake_last_raw) {
      brake_stable_count = 0;
      brake_last_raw = raw;
    } else if (raw != brake_pressed) {
      uint8_t need = raw ? BRAKE_DEBOUNCE_PRESS_SAMPLES
                         : BRAKE_DEBOUNCE_RELEASE_SAMPLES;
      if (++brake_stable_count >= need) {
        brake_pressed = raw;
        brake_stable_count = 0;
      }
    }
  }
  brake_pressure_telem = brake_pressed ? 3.3f : 0.0f;
  digital_data.brake_led =
      brake_pressed || (regen_in >= REGEN_BRAKE_LIGHT_THRESHOLD);

#ifdef DEBUG_PRINTS
  {
    static uint32_t lastAgentLogMs = 0;
    uint32_t nowMs = millis();
    if (nowMs - lastAgentLogMs >= 1000) {
      lastAgentLogMs = nowMs;
      uint32_t dig = (uint32_t)digitalRead(BRAKE_TELEM);
      uint32_t mode = (GPIOA->MODER >> 0) & 0x3U;
      uint32_t pupd = (GPIOA->PUPDR >> 0) & 0x3U; // 2 = pulldown
      uint32_t idr = (GPIOA->IDR >> 0) & 0x1U;
      agentDebugLogBrakeDigital(dig, mode, pupd, idr);
    }
  }
#endif
#endif
}

void set_direction(bool dir) {
  digitalWrite(MCU_DIR, dir);
  digital_data.direction = dir;
}

void set_eco_mode(bool eco) {
  digitalWrite(MCU_ECO, eco);
  digital_data.eco_mode = eco;
}

void writeAccOut(float newAccOut) {
  // Clamp to valid DAC command range before converting to integer counts.
  if (newAccOut < 0.0f) newAccOut = 0.0f;
  else if (newAccOut > 1.0f) newAccOut = 1.0f;

  acc_out = newAccOut;
  // printf("acc_out: %f\n", acc_out);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint32_t)(acc_out * 4095.0f));
}

void writeRegenBrake(float newRegenBrake) {
  // Clamp to valid DAC command range before converting to integer counts.
  if (newRegenBrake < 0.0f) newRegenBrake = 0.0f;
  else if (newRegenBrake > 1.0f) newRegenBrake = 1.0f;

  regen_brake = newRegenBrake;
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)(regen_brake * 4095.0f));
}
