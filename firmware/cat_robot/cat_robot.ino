/* ============================================================================
 *  Autonomous Cat Robot  -  ESP32 WROOM-32
 *
 *  Components used (minimal build):
 *    - ESP32 WROOM-32
 *    - TB6612FNG dual motor driver  -> 2x BO motors
 *    - HC-SR04 ultrasonic           -> distance / Q-learning state
 *    - IR sensor (pointed DOWN)      -> tabletop edge / anti-fall safety
 *    - SSD1306 0.96" OLED (I2C)      -> face / expression
 *
 *  Brain: 4-state x 3-action Q-table, pre-trained in sim (qtable.h), kept
 *  learning on-device and saved to NVS flash every 60s.
 *
 *  Libraries (Arduino Library Manager):
 *    Adafruit SSD1306, Adafruit GFX
 *  (Preferences is built into the ESP32 core.)
 *
 *  See docs/wiring.md for the pin map and the HC-SR04 ECHO voltage divider.
 * ========================================================================== */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include "qtable.h"          // const float Q_INIT[4][3], Q_STATES, Q_ACTIONS

// ----------------------------------------------------------------------------
// Pin map  (matches docs/wiring.md)
// ----------------------------------------------------------------------------
#define PIN_SDA      21      // I2C OLED
#define PIN_SCL      22

#define PIN_TRIG      5      // HC-SR04 trigger
#define PIN_ECHO     18      // HC-SR04 echo  (THROUGH 1k/2k DIVIDER -> 3.3V)

#define PIN_IR       34      // IR edge sensor (input-only pin)

// TB6612FNG
#define PIN_STBY      4
#define PIN_AIN1     16      // left motor (A)
#define PIN_AIN2     17
#define PIN_PWMA     25
#define PIN_BIN1     27      // right motor (B)
#define PIN_BIN2     26
#define PIN_PWMB     33

// IR module logic: many modules read LOW when a surface is detected and HIGH
// when there's nothing below (edge / drop). If your cat thinks it's ALWAYS at
// an edge, flip this to LOW.
#define IR_EDGE_LEVEL  HIGH

// ----------------------------------------------------------------------------
// OLED
// ----------------------------------------------------------------------------
#define SCREEN_W 128
#define SCREEN_H 64
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

// ----------------------------------------------------------------------------
// Q-learning
// ----------------------------------------------------------------------------
enum { S_DANGER = 0, S_CLOSE, S_COMFORTABLE, S_CLEAR };   // states
enum { A_FORWARD = 0, A_TURN, A_REVERSE };                // actions

float Q[Q_STATES][Q_ACTIONS];

const float ALPHA   = 0.20f;   // learning rate
const float GAMMA   = 0.90f;   // discount
float       epsilon = 0.15f;   // exploration (starts modest, decays to floor)
const float EPS_MIN = 0.05f;
const float EPS_DECAY = 0.9995f;

// distance thresholds (cm)
const float T_DANGER = 6.0f;
const float T_CLOSE  = 18.0f;
const float T_COMF   = 40.0f;

// ----------------------------------------------------------------------------
// Motors
// ----------------------------------------------------------------------------
// BO/TT motors are often rated 3-6V but the pack is ~7.4V, so we CAP the PWM
// duty so the average motor voltage stays in spec. Tune DRIVE_SPEED on the
// bench: lower it if the cat is too fast / jittery.
const int MAX_DUTY    = 160;   // 0..255 hard ceiling (~63% of 7.4V ~= 4.7V)
const int DRIVE_SPEED = 150;   // forward/reverse speed
const int TURN_SPEED  = 140;   // spin speed

// ----------------------------------------------------------------------------
// Timing / behavior
// ----------------------------------------------------------------------------
const unsigned long TICK_MS      = 500;    // one Q step every 500ms
const unsigned long SAVE_MS      = 60000;  // persist table every 60s
const unsigned long IDLE_MS      = 45000;  // doze after 45s of no change
const unsigned long STARTLE_MS   = 1500;   // startled face/spin duration
const float STARTLE_DROP_CM      = 15.0f;  // sudden approach this much -> flinch
const float IDLE_MOVE_CM         = 8.0f;   // change smaller than this = "still"

// ----------------------------------------------------------------------------
// State variables
// ----------------------------------------------------------------------------
Preferences prefs;

int   prevState  = S_CLEAR;
int   prevAction = -1;       // -1 = no action taken yet
float lastDist   = 100.0f;   // for startle comparison
float idleAnchor = 100.0f;   // distance when "stillness" started

unsigned long lastTick    = 0;
unsigned long lastSave    = 0;
unsigned long lastChange  = 0;
unsigned long startleUntil = 0;
unsigned long stepCount   = 0;

// face ids
enum { FACE_SOFT, FACE_WIDE, FACE_NARROW, FACE_STARTLED, FACE_SLEEPY, FACE_BLINK };

// ============================================================================
//  SENSORS
// ============================================================================
float readDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long dur = pulseIn(PIN_ECHO, HIGH, 25000UL);   // 25ms timeout (~4m)
  if (dur == 0) return 200.0f;                   // no echo -> treat as far/clear
  float cm = (dur * 0.0343f) / 2.0f;
  if (cm < 2.0f)   cm = 2.0f;
  if (cm > 200.0f) cm = 200.0f;
  return cm;
}

int stateFromDistance(float cm) {
  if (cm < T_DANGER) return S_DANGER;
  if (cm < T_CLOSE)  return S_CLOSE;
  if (cm < T_COMF)   return S_COMFORTABLE;
  return S_CLEAR;
}

bool atEdge() {                                  // IR cliff / table-edge check
  return digitalRead(PIN_IR) == IR_EDGE_LEVEL;
}

// ============================================================================
//  MOTORS  (TB6612FNG)
// ============================================================================
int cap(int duty) { return duty > MAX_DUTY ? MAX_DUTY : duty; }

void driveMotor(int in1, int in2, int pwmPin, int speed, bool forward) {
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW  : HIGH);
  analogWrite(pwmPin, cap(speed));
}

void stopMotors() {
  analogWrite(PIN_PWMA, 0);
  analogWrite(PIN_PWMB, 0);
}

void goForward() {
  driveMotor(PIN_AIN1, PIN_AIN2, PIN_PWMA, DRIVE_SPEED, true);
  driveMotor(PIN_BIN1, PIN_BIN2, PIN_PWMB, DRIVE_SPEED, true);
}

void goReverse() {
  driveMotor(PIN_AIN1, PIN_AIN2, PIN_PWMA, DRIVE_SPEED, false);
  driveMotor(PIN_BIN1, PIN_BIN2, PIN_PWMB, DRIVE_SPEED, false);
}

void spinTurn() {                                 // turn in place, random side
  bool right = random(0, 2);
  driveMotor(PIN_AIN1, PIN_AIN2, PIN_PWMA, TURN_SPEED, right);   // left wheel
  driveMotor(PIN_BIN1, PIN_BIN2, PIN_PWMB, TURN_SPEED, !right);  // right wheel
}

void doAction(int a) {
  switch (a) {
    case A_FORWARD: goForward(); break;
    case A_TURN:    spinTurn();  break;
    case A_REVERSE: goReverse(); break;
  }
}

// ============================================================================
//  REWARD  (mirrors sim/train.py so on-device learning matches the pre-train)
// ============================================================================
float rewardFor(int action, int newState) {
  switch (newState) {
    case S_DANGER:
      return (action == A_FORWARD) ? -11.0f : -6.0f;   // -6, extra -5 if forward
    case S_CLOSE:
      return (action == A_FORWARD) ? -1.0f : 1.0f;
    case S_COMFORTABLE:
      if (action == A_FORWARD) return  5.0f;           // follow / hold the gap
      if (action == A_REVERSE) return -3.0f;           // don't back out
      return 0.0f;
    default: // S_CLEAR
      return (action == A_FORWARD) ? 1.0f : 0.0f;      // mild roaming
  }
}

int bestAction(int s) {
  int best = 0;
  for (int a = 1; a < Q_ACTIONS; a++)
    if (Q[s][a] > Q[s][best]) best = a;
  return best;
}

int chooseAction(int s) {
  if ((random(0, 10000) / 10000.0f) < epsilon) return random(0, Q_ACTIONS);
  return bestAction(s);
}

// ============================================================================
//  NVS  (persist the 48-byte table)
// ============================================================================
void loadTable() {
  prefs.begin("cat", false);
  size_t want = sizeof(Q);
  if (prefs.getBytesLength("qtable") == want) {
    prefs.getBytes("qtable", Q, want);            // resume what it learned
  } else {
    memcpy(Q, Q_INIT, want);                      // first boot -> pre-trained
  }
  prefs.end();
}

void saveTable() {
  prefs.begin("cat", false);
  prefs.putBytes("qtable", Q, sizeof(Q));
  prefs.end();
}

// ============================================================================
//  FACE  (procedural eye bitmaps on the SSD1306)
// ============================================================================
const int LX = 40, RX = 88, EY = 32;             // eye centers

void drawEye(int cx, int cy, int w, int h, int pr, int pdx, int pdy) {
  int r = min(w, h) / 3;
  display.fillRoundRect(cx - w / 2, cy - h / 2, w, h, r, SSD1306_WHITE);
  if (pr > 0) display.fillCircle(cx + pdx, cy + pdy, pr, SSD1306_BLACK);  // pupil
}

void renderFace(int face) {
  display.clearDisplay();
  switch (face) {
    case FACE_SOFT:                              // following / content
      drawEye(LX, EY, 30, 24, 6, 0, 0);
      drawEye(RX, EY, 30, 24, 6, 0, 0);
      break;
    case FACE_WIDE:                              // curious / alert (open space)
      drawEye(LX, EY, 32, 32, 5, 0, 0);
      drawEye(RX, EY, 32, 32, 5, 0, 0);
      break;
    case FACE_NARROW:                            // danger / focused squint
      drawEye(LX, EY, 32, 12, 4, 0, 0);
      drawEye(RX, EY, 32, 12, 4, 0, 0);
      break;
    case FACE_STARTLED:                          // huge dilated pupils
      drawEye(LX, EY, 36, 36, 12, 0, 0);
      drawEye(RX, EY, 36, 36, 12, 0, 0);
      break;
    case FACE_SLEEPY:                            // half-closed, dozing
      drawEye(LX, EY + 6, 30, 7, 0, 0, 0);
      drawEye(RX, EY + 6, 30, 7, 0, 0, 0);
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(104, 6);
      display.print("z");
      display.setCursor(112, 0);
      display.print("Z");
      break;
    case FACE_BLINK:                             // closed for one frame
      drawEye(LX, EY, 30, 4, 0, 0, 0);
      drawEye(RX, EY, 30, 4, 0, 0, 0);
      break;
  }
  display.display();
}

int faceForState(int s) {
  switch (s) {
    case S_DANGER:      return FACE_NARROW;
    case S_CLOSE:       return FACE_WIDE;
    case S_COMFORTABLE: return FACE_SOFT;
    default:            return FACE_WIDE;        // clear -> curious
  }
}

// ============================================================================
//  SAFETY OVERRIDES  (outside the Q-loop, like a real cat's reflexes)
// ============================================================================
void edgeReflex() {                              // IR sees a drop: don't fall!
  renderFace(FACE_STARTLED);
  stopMotors();
  delay(80);
  goReverse();
  delay(350);
  spinTurn();
  delay(300);
  stopMotors();
  lastChange = millis();                         // count as activity
  lastDist   = readDistanceCm();
}

void startleReflex() {                           // something rushed at the face
  startleUntil = millis() + STARTLE_MS;
  renderFace(FACE_STARTLED);
  goReverse();
  delay(250);
  spinTurn();
  delay(250);
  stopMotors();
}

// ============================================================================
//  Q-LEARNING TICK  (every 500ms)
// ============================================================================
void qTick() {
  float dist  = readDistanceCm();
  int   state = stateFromDistance(dist);

  // --- startle: sudden large drop in distance = fast approach ---------------
  if ((lastDist - dist) > STARTLE_DROP_CM) {
    lastDist   = dist;
    lastChange = millis();
    startleReflex();
    return;
  }

  // --- learn from the result of the PREVIOUS action ------------------------
  if (prevAction >= 0) {
    float r = rewardFor(prevAction, state);
    float maxNext = Q[state][bestAction(state)];
    Q[prevState][prevAction] += ALPHA * (r + GAMMA * maxNext - Q[prevState][prevAction]);
  }

  // --- idle / dozing detection --------------------------------------------
  if (fabs(dist - idleAnchor) > IDLE_MOVE_CM) {
    idleAnchor = dist;
    lastChange = millis();
  }
  bool dozing = (millis() - lastChange) > IDLE_MS;

  if (dozing) {
    stopMotors();
    renderFace(FACE_SLEEPY);
  } else {
    // --- choose + take the next action ------------------------------------
    int a = chooseAction(state);
    doAction(a);
    prevState  = state;
    prevAction = a;

    // occasional blink while content, else the state's face
    if (state == S_COMFORTABLE && random(0, 5) == 0) renderFace(FACE_BLINK);
    else renderFace(faceForState(state));
  }

  // decay exploration toward the floor
  epsilon = max(EPS_MIN, epsilon * EPS_DECAY);
  stepCount++;
  lastDist = dist;
}

// ============================================================================
//  SETUP / LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_IR,   INPUT);

  pinMode(PIN_STBY, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);                  // enable the driver
  stopMotors();

  randomSeed(esp_random());

  Wire.begin(PIN_SDA, PIN_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 not found - check I2C wiring/address");
    while (true) delay(1000);
  }

  loadTable();

  unsigned long now = millis();
  lastTick = lastSave = lastChange = now;
  lastDist = idleAnchor = readDistanceCm();

  renderFace(FACE_WIDE);                          // wake up curious
}

void loop() {
  // Cliff safety runs every loop (fast) - falling off the table is the
  // worst outcome, so it gets the highest priority and pre-empts everything.
  if (atEdge()) {
    edgeReflex();
    return;
  }

  // Hold the startle pose until its timer expires.
  if (millis() < startleUntil) return;

  if (millis() - lastTick >= TICK_MS) {
    lastTick = millis();
    qTick();
  }

  if (millis() - lastSave >= SAVE_MS) {
    lastSave = millis();
    saveTable();                                  // remember across power cycles
    Serial.printf("[save] step=%lu eps=%.3f  Q: D[%.1f %.1f %.1f] CL[%.1f %.1f %.1f] CF[%.1f %.1f %.1f] CR[%.1f %.1f %.1f]\n",
      stepCount, epsilon,
      Q[0][0],Q[0][1],Q[0][2], Q[1][0],Q[1][1],Q[1][2],
      Q[2][0],Q[2][1],Q[2][2], Q[3][0],Q[3][1],Q[3][2]);
  }
}
