# Autonomous Cat Robot 🐱

A tabletop "cat" robot on an ESP32 that learns to navigate with a tiny Q-learning
table (4 states × 3 actions = 12 numbers), reacts to bumps with a hard MPU6050
startle reflex, and shows mood through an OLED face, servo, and LED.

## Design summary

- **Brain:** Q-table pre-trained in a Python simulator, exported as a C array, then
  flashed into the ESP32 where it keeps learning online and saves to NVS flash.
- **States (ultrasonic distance):** `0 Danger <6cm`, `1 Close 6–18cm`,
  `2 Comfortable 18–40cm`, `3 Clear >40cm`.
- **Actions:** `0 Forward`, `1 Turn`, `2 Reverse`.
- **Loop:** every ~500 ms → read distance → choose action (ε-greedy) → drive motors →
  read new distance → compute reward → Bellman update → repeat. Save table every 60 s.
- **Hard reflexes (outside the Q-loop):**
  - MPU6050 acceleration spike → reverse-and-turn + startled face.
  - IR edge sensor sees no table → immediate stop/reverse (anti-fall safety).
- **Expression layer:** deterministic state machine maps behavior → OLED eyes + servo
  pose + LED color. Does not touch learning.

## Components used (from your kit)
ESP32 WROOM-32 · TB6612FNG motor driver · 2× BO motors + wheels + castor ·
HC-SR04 ultrasonic · IR sensor (edge/anti-fall) · MPU6050 · SSD1306 0.96" OLED ·
SG90 servo · LED(s) · 2S Li-ion pack + switch.

**Add to BOM:** one 5V buck converter (MP1584 or LM2596) to power the ESP32/servo/
HC-SR04 from the 7.4V pack. See `docs/wiring.md`.

## Repo structure
```
.
├── README.md                 ← this file
├── docs/
│   └── wiring.md             ← pin map, power, dividers, BOM note
├── sim/                      ← Python Q-learning trainer (STEP 1)
│   ├── train.py              ← environment + training loop
│   └── qtable.h              ← generated: trained table as C array (output)
└── firmware/
    └── cat_robot/
        └── cat_robot.ino     ← Arduino sketch (STEP 2+)
```

## Build order
1. **Simulator** (`sim/train.py`) — model the distance dynamics + reward shaping,
   train until the 4×3 table converges, print it, and export `sim/qtable.h`.
   *Goal: you can watch the cat's "policy" make sense before any hardware.*
2. **Firmware skeleton** — pin definitions, I2C/OLED bring-up, a "hello eyes" face so
   you confirm wiring works.
3. **Sensors** — HC-SR04 distance reads, IR edge read, MPU6050 init + INT.
4. **Motors** — TB6612 forward/turn/reverse helpers with PWM cap.
5. **Q-loop** — drop in `qtable.h`, implement ε-greedy + Bellman + NVS save/load.
6. **Reflexes** — MPU startle override + IR anti-fall stop.
7. **Expression layer** — map behavior states to OLED bitmaps + servo + LED.
8. **Tuning** — thresholds, reward weights, PWM cap, timing on the real robot.

## Arduino libraries needed (Library Manager)
- `Adafruit SSD1306` + `Adafruit GFX`
- `Adafruit MPU6050` + `Adafruit Unified Sensor`
- ESP32 `Preferences` (built into ESP32 core) for NVS
- Servo: `ESP32Servo`
- (HC-SR04, IR, TB6612 are driven with plain GPIO — no library needed.)

## Status
- [x] Plan + structure + wiring
- [x] Step 1: Python simulator + exported table (`sim/qtable.h`)
- [x] Step 2-7: firmware (`firmware/cat_robot/cat_robot.ino`) - minimal build
      (ESP32, TB6612, 2x BO motors, HC-SR04, IR edge, OLED; no servo/MPU/LED)
- [ ] Step 8: tune thresholds/speeds on the real robot

## Reward shaping (tuned in sim)
Danger -6 (extra -5 if driven into). Close: Forward -1, else +1. Comfortable:
Forward +5, Reverse -3, Turn 0 (this makes the sweet spot the value PEAK so the
cat follows instead of fleeing). Clear: Forward +1, else 0. α=0.2, γ=0.9, ε 0.30→0.05.
Converged greedy policy: Danger->Turn, Close->Turn, Comfortable->Forward, Clear->Forward.
