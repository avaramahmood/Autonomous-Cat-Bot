# Wiring & Pin Map — Autonomous Cat Robot (ESP32 WROOM-32)

## Power architecture

```
2S Li-ion pack (7.4V nominal, 8.4V full, 6.0V empty)
        |
   [ Switch ]
        |
        +---------------------------+
        |                           |
   TB6612FNG VM (motors)      Buck converter -> 5V rail
   (7.4V direct, OK)          (MP1584 / LM2596)  *** ADD TO BOM ***
                                    |
                    +---------------+----------------+
                    |               |                |
              ESP32 VIN (5V)   HC-SR04 VCC (5V)   SG90 servo (5V)
                    |
              ESP32 3V3 pin (regulated on board) -> 3.3V rail
                    |
        +-----------+-----------+-----------+
        |           |           |           |
   MPU6050 VCC  OLED VCC   TB6612 VCC   IR sensor VCC
   (3.3V)       (3.3V)     (logic 3.3V) (3.3V)
```

**Critical rules**
- NEVER feed 7.4–8.4V into the ESP32 VIN. Use the 5V buck output.
- TB6612FNG `VM` = battery (7.4V). TB6612FNG `VCC` = 3.3V logic from ESP32.
- HC-SR04 `ECHO` outputs 5V — **must** go through a divider to the ESP32 (3.3V).
- Servo gets its own 5V from the buck (not the ESP32 3V3 pin) to avoid brownouts.
- Tie ALL grounds together (battery −, buck GND, ESP32 GND, TB6612 GND, sensors).

**Charging note:** the TP4056 ("TB4056") is a *single-cell (1S)* charger. It cannot
charge a 2S pack as a unit. Charge each cell individually, or add a 2S balance
charger/BMS. (Does not affect firmware.)

## HC-SR04 ECHO voltage divider
```
ECHO ---[ 1kΩ ]---+--- GPIO18 (ESP32)
                  |
               [ 2kΩ ]
                  |
                 GND
```
(1k/2k gives 5V * 2/3 ≈ 3.3V. Use values from your resistor box, ratio ~1:2.)

## Pin map

| ESP32 GPIO | Connects to            | Notes |
|-----------:|------------------------|-------|
| 21         | I2C SDA (OLED + MPU6050) | shared bus |
| 22         | I2C SCL (OLED + MPU6050) | shared bus |
| 5          | HC-SR04 TRIG           | output |
| 18         | HC-SR04 ECHO           | **via 1k/2k divider** |
| 34         | IR edge sensor OUT     | input-only pin (digital), tabletop safety |
| 35         | MPU6050 INT            | input-only pin, startle interrupt |
| 4          | TB6612 STBY            | high = driver enabled |
| 16         | TB6612 AIN1            | left motor dir |
| 17         | TB6612 AIN2            | left motor dir |
| 25         | TB6612 PWMA            | left motor speed (LEDC) |
| 27         | TB6612 BIN1            | right motor dir |
| 26         | TB6612 BIN2            | right motor dir |
| 33         | TB6612 PWMB            | right motor speed (LEDC) |
| 13         | SG90 servo signal      | LEDC 50Hz (tail/head tilt) |
| 14         | LED R                  | status / expression |
| 19         | LED G                  | status / expression |
| 23         | LED B                  | status / expression |
| 32         | Push button (mode/reset learning) | input w/ pullup |

Pins avoided on purpose: 0, 2, 12, 15 (boot strapping), 6–11 (flash).
GPIO 34 & 35 are input-only — used only for the IR and MPU INT inputs.

## I2C addresses
- SSD1306 OLED: `0x3C`
- MPU6050: `0x68`

## TB6612FNG motor truth table (per channel)
| IN1 | IN2 | Result   |
|-----|-----|----------|
| 1   | 0   | forward  |
| 0   | 1   | reverse  |
| 1   | 1   | brake    |
| 0   | 0   | coast    |
Speed = PWM duty on PWMx. STBY must be HIGH.

## Motor safety
BO/TT motors are commonly rated 3–6V. The pack is 7.4V, so firmware caps PWM duty
(`MAX_DUTY`) so the *average* motor voltage stays in spec. Start at ~60–70% and tune.
