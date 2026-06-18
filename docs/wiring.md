# Wiring & Pin Map — Autonomous Cat Robot (ESP32 WROOM-32)

## Power architecture (NO buck converter — uses the ESP32's onboard regulator)

```
2S Li-ion pack (7.4V nominal, 8.4V full, 6.0V empty)
        |
   [ Switch ]
        |
        +---------------------------+
        |                           |
   TB6612FNG VM (motors)      ESP32 VIN pin
   (7.4V direct, OK)          (onboard AMS1117 regulator -> 3.3V)
                                    |
                              ESP32 3V3 pin -> 3.3V rail
                                    |
        +-----------+-----------+-----------+
        |           |           |           |
   OLED VCC    IR sensor   HC-SR04 VCC   TB6612 VCC
   (3.3V)      VCC (3.3V)  (3.3V)        (logic 3.3V)
```

The ESP32 dev board's built-in AMS1117 IS our regulator — no external buck needed.
The TB6612FNG has NO power output of its own (its VCC is an input you feed), so it
cannot power the ESP32.

**Critical rules**
- ESP32 VIN <- 7.4V battery. The onboard AMS1117 makes 3.3V. This is allowed here
  ONLY because we never enable Wi-Fi/BT (low current keeps the regulator from
  overheating). The regulator WILL run warm — that's expected.
- Keep loads on the 3V3 pin light (OLED + IR + HC-SR04 + TB6612 logic only).
- TB6612FNG `VM` = battery (7.4V). TB6612FNG `VCC` = 3.3V logic from ESP32 3V3.
- HC-SR04 runs at 3.3V here, so its ECHO is 3.3V — wire it STRAIGHT to GPIO18,
  NO voltage divider needed.
- Motors get their power from VM (7.4V) via the driver; firmware caps PWM duty so
  the 3-6V BO motors aren't over-driven.
- Tie ALL grounds together (battery −, ESP32 GND, TB6612 GND, every sensor GND).

**HC-SR04 at 3.3V:** works fine at the short tabletop ranges we use (<40cm matters).
Max long-range accuracy drops a little at 3.3V; irrelevant for this robot. If your
particular module refuses to trigger at 3.3V, that's the one place you'd want a 5V
source — but the build does not require it.

**Charging note:** the TP4056 ("TB4056") is a *single-cell (1S)* charger. It cannot
charge a 2S pack as a unit. Charge each cell individually, or add a 2S balance
charger/BMS. (Does not affect firmware.)

## Pin map (minimal build — only the components in use)

| ESP32 GPIO | Connects to       | Notes |
|-----------:|-------------------|-------|
| 21         | OLED SDA          | I2C |
| 22         | OLED SCL          | I2C |
| 5          | HC-SR04 TRIG      | output |
| 18         | HC-SR04 ECHO      | direct (sensor on 3.3V — NO divider) |
| 34         | IR edge sensor OUT | input-only pin, tabletop anti-fall |
| 4          | TB6612 STBY       | high = driver enabled |
| 16         | TB6612 AIN1       | left motor dir |
| 17         | TB6612 AIN2       | left motor dir |
| 25         | TB6612 PWMA       | left motor speed |
| 27         | TB6612 BIN1       | right motor dir |
| 26         | TB6612 BIN2       | right motor dir |
| 33         | TB6612 PWMB       | right motor speed |

Pins avoided on purpose: 0, 2, 12, 15 (boot strapping), 6–11 (flash).
GPIO 34 is input-only — used only for the IR input.
Not used in this build: servo, MPU6050, LEDs, buttons, DHT11, potentiometer.

## I2C addresses
- SSD1306 OLED: `0x3C`

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
