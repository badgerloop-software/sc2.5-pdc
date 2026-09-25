# sc2-pdc

Firmware for Nucleo-L432KC based Power Distribution and Controls board.

> **Repository origin:** This Car 2.5 repository was created from the [`v2.0.00`](https://github.com/badgerloop-software/sc2-pdc/tree/v2.0.00) tag of the original [`badgerloop-software/sc2-pdc`](https://github.com/badgerloop-software/sc2-pdc) repository. The starting commit is `db26e48`.

## Build

1. Run `git submodule update --init`.
2. Run `pio run`.

## Modules

1. `main` starts the board, optional debug mode, and the CAN transmit loop.
2. `IOManagement` reads GPIO and ADC values. It writes accel and regen DAC outputs.
3. `speed_calc` counts MCU speed pulses and sets `rpm` and `mph`.
4. `motor_control` runs the PARK, IDLE, FORWARD, and REVERSE state machine.
5. `canPDC` receives driver inputs on CAN. It transmits telem on CAN.

## Build flags

Set these flags in `include/const.h`. Set `DEBUG_TECHNIQUE` in `main.cpp`.

| Flag              | Role                                                |
| ----------------- | --------------------------------------------------- |
| `DEBUG_PRINTS`    | Print status on Serial. This flag is on by default. |
| `TEST_MODE`       | Use the CAN bounce test board path.                 |
| `DEBUG_TECHNIQUE` | `0` is production. `1` sends random CAN echo data.  |

## Parking brake

The parking brake sensor is not on the car. The firmware sets `park_brake` to false. The controller leaves PARK and runs.

## Pins

| Signal          | Pin                     |
| --------------- | ----------------------- |
| Accel DAC       | PA5                     |
| Regen DAC       | PA4                     |
| Brake pressure  | PA0 (ADC1 channel 5)    |
| MCU speed pulse | PA8                     |
| Direction       | PB7                     |
| Eco             | PB1                     |

## CAN map

Receive from the steering wheel:

- `0x300` direction
- `0x301` regen
- `0x302` throttle
- `0x303` eco or power mode

Transmit:

- `0x200` through `0x208`: accel out, regen, LV telem, brake, digital pack, mph
- `0x206`: brake pressure in PSI as little-endian float32, clamped to 0–2000
- `0x207` bit 5: brake-light Boolean

The brake pressure sensor outputs 0.5–4.5 V for 0–2000 PSI. The external
voltage divider maps 4.5 V from the sensor to 3.3 V at PA0. Brake pressure at
or above 100 PSI asserts the brake input and pressure at or below 75 PSI
releases it. The brake light is on while that input is asserted or requested
regen is at least 5%.
