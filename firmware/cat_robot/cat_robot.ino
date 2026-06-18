/*
 * cat_robot - ESP32 tabletop pet
 * TB6612 + 2 BO motors, HC-SR04, IR edge sensor, SSD1306 OLED.
 * 4x3 Q-table (qtable.h) trained offline, keeps learning + saves to NVS.
 *
 * Power: 2S pack -> ESP32 VIN + TB6612 VM. Rest off the 3V3 pin.
 * Keep WiFi/BT off or the onboard regulator cooks. See docs/wiring.md.
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include "qtable.h"

// --- pins ---
#define PIN_SDA      21
#define PIN_SCL      22

#define PIN_TRIG      5
#define PIN_ECHO     18      // sensor at 3.3V, no divider needed

#define PIN_IR       34      // input-only pin, IR points down at the table

// TB6612
#define PIN_STBY      4
#define PIN_AIN1     16      // left
#define PIN_AIN2     17
#define PIN_PWMA     25
#define PIN_BIN1     27      // right
#define PIN_BIN2     26
#define PIN_PWMB     33

// flip to LOW if the cat thinks it's always at an edge (modules vary)
#define IR_EDGE_LEVEL  HIGH

#define SCREEN_W 128
#define SCREEN_H 64
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

// --- Q-learning ---
enum { S_DANGER = 0, S_CLOSE, S_COMFORTABLE, S_CLEAR };
enum { A_FORWARD = 0, A_TURN, A_REVERSE };

float Q[Q_STATES][Q_ACTIONS];

const float ALPHA   = 0.20f;
const float GAMMA   = 0.90f;
float       epsilon = 0.15f;   // decays toward EPS_MIN
const float EPS_MIN = 0.05f;
const float EPS_DECAY = 0.9995f;

// state cutoffs, cm
const float T_DANGER = 6.0f;
const float T_CLOSE  = 18.0f;
const float T_COMF   = 40.0f;

// --- motors ---
// BO motors are 3-6V, pack is 7.4V, so cap the duty. drop DRIVE_SPEED if jittery.
const int MAX_DUTY    = 160;   // ~4.7V average off 7.4V
const int DRIVE_SPEED = 150;
const int TURN_SPEED  = 140;

// --- timing ---
const unsigned long TICK_MS      = 500;
const unsigned long SAVE_MS      = 60000;
const unsigned long IDLE_MS      = 45000;   // doze threshold
const unsigned long STARTLE_MS   = 1500;
const float STARTLE_DROP_CM      = 15.0f;   // this much closer in one tick = flinch
const float IDLE_MOVE_CM         = 8.0f;    // smaller change = "didn't move"

Preferences prefs;

int   prevState  = S_CLEAR;
int   prevAction = -1;
float lastDist   = 100.0f;
float idleAnchor = 100.0f;

unsigned long lastTick    = 0;
unsigned long lastSave    = 0;
unsigned long lastChange  = 0;
unsigned long startleUntil = 0;
unsigned long stepCount   = 0;

enum { FACE_SOFT, FACE_WIDE, FACE_NARROW, FACE_STARTLED, FACE_SLEEPY, FACE_BLINK };

// --- sensors ---
float readDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long dur = pulseIn(PIN_ECHO, HIGH, 25000UL);   // ~4m timeout
  if (dur == 0) return 200.0f;                   // no echo, call it far
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

bool atEdge() {
  return digitalRead(PIN_IR) == IR_EDGE_LEVEL;
}

// --- motors ---
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

void spinTurn() {
  bool right = random(0, 2);                      // pick a side
  driveMotor(PIN_AIN1, PIN_AIN2, PIN_PWMA, TURN_SPEED, right);
  driveMotor(PIN_BIN1, PIN_BIN2, PIN_PWMB, TURN_SPEED, !right);
}

void doAction(int a) {
  switch (a) {
    case A_FORWARD: goForward(); break;
    case A_TURN:    spinTurn();  break;
    case A_REVERSE: goReverse(); break;
  }
}

// --- reward (kept identical to sim/train.py) ---
float rewardFor(int action, int newState) {
  switch (newState) {
    case S_DANGER:
      return (action == A_FORWARD) ? -11.0f : -6.0f;
    case S_CLOSE:
      return (action == A_FORWARD) ? -1.0f : 1.0f;
    case S_COMFORTABLE:
      if (action == A_FORWARD) return  5.0f;
      if (action == A_REVERSE) return -3.0f;
      return 0.0f;
    default: // S_CLEAR
      return (action == A_FORWARD) ? 1.0f : 0.0f;
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

// --- NVS (48-byte table) ---
void loadTable() {
  prefs.begin("cat", false);
  size_t want = sizeof(Q);
  if (prefs.getBytesLength("qtable") == want)
    prefs.getBytes("qtable", Q, want);            // resume
  else
    memcpy(Q, Q_INIT, want);                      // first boot
  prefs.end();
}

void saveTable() {
  prefs.begin("cat", false);
  prefs.putBytes("qtable", Q, sizeof(Q));
  prefs.end();
}

// --- face ---
const int LX = 40, RX = 88, EY = 32;             // eye centers

void drawEye(int cx, int cy, int w, int h, int pr, int pdx, int pdy) {
  int r = min(w, h) / 3;
  display.fillRoundRect(cx - w / 2, cy - h / 2, w, h, r, SSD1306_WHITE);
  if (pr > 0) display.fillCircle(cx + pdx, cy + pdy, pr, SSD1306_BLACK);
}

void renderFace(int face) {
  display.clearDisplay();
  switch (face) {
    case FACE_SOFT:                              // content / following
      drawEye(LX, EY, 30, 24, 6, 0, 0);
      drawEye(RX, EY, 30, 24, 6, 0, 0);
      break;
    case FACE_WIDE:                              // alert / curious
      drawEye(LX, EY, 32, 32, 5, 0, 0);
      drawEye(RX, EY, 32, 32, 5, 0, 0);
      break;
    case FACE_NARROW:                            // squint / danger
      drawEye(LX, EY, 32, 12, 4, 0, 0);
      drawEye(RX, EY, 32, 12, 4, 0, 0);
      break;
    case FACE_STARTLED:                          // big pupils
      drawEye(LX, EY, 36, 36, 12, 0, 0);
      drawEye(RX, EY, 36, 36, 12, 0, 0);
      break;
    case FACE_SLEEPY:                            // half shut + zZ
      drawEye(LX, EY + 6, 30, 7, 0, 0, 0);
      drawEye(RX, EY + 6, 30, 7, 0, 0, 0);
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(104, 6); display.print("z");
      display.setCursor(112, 0); display.print("Z");
      break;
    case FACE_BLINK:
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
    default:            return FACE_WIDE;
  }
}

// --- reflexes (run outside the Q-loop) ---
void edgeReflex() {                              // IR sees a drop, back away
  renderFace(FACE_STARTLED);
  stopMotors();
  delay(80);
  goReverse();
  delay(350);
  spinTurn();
  delay(300);
  stopMotors();
  lastChange = millis();
  lastDist   = readDistanceCm();
}

void startleReflex() {                           // something rushed in
  startleUntil = millis() + STARTLE_MS;
  renderFace(FACE_STARTLED);
  goReverse();
  delay(250);
  spinTurn();
  delay(250);
  stopMotors();
}

void qTick() {
  float dist  = readDistanceCm();
  int   state = stateFromDistance(dist);

  if ((lastDist - dist) > STARTLE_DROP_CM) {     // fast approach -> flinch
    lastDist   = dist;
    lastChange = millis();
    startleReflex();
    return;
  }

  // learn from the previous action's outcome
  if (prevAction >= 0) {
    float r = rewardFor(prevAction, state);
    float maxNext = Q[state][bestAction(state)];
    Q[prevState][prevAction] += ALPHA * (r + GAMMA * maxNext - Q[prevState][prevAction]);
  }

  // reset the idle timer if it actually moved
  if (fabs(dist - idleAnchor) > IDLE_MOVE_CM) {
    idleAnchor = dist;
    lastChange = millis();
  }

  if ((millis() - lastChange) > IDLE_MS) {       // dozing
    stopMotors();
    renderFace(FACE_SLEEPY);
  } else {
    int a = chooseAction(state);
    doAction(a);
    prevState  = state;
    prevAction = a;
    if (state == S_COMFORTABLE && random(0, 5) == 0) renderFace(FACE_BLINK);
    else renderFace(faceForState(state));
  }

  epsilon = max(EPS_MIN, epsilon * EPS_DECAY);
  stepCount++;
  lastDist = dist;
}

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
  digitalWrite(PIN_STBY, HIGH);
  stopMotors();

  randomSeed(esp_random());

  Wire.begin(PIN_SDA, PIN_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("no SSD1306 - check I2C wiring/address");
    while (true) delay(1000);
  }

  loadTable();

  unsigned long now = millis();
  lastTick = lastSave = lastChange = now;
  lastDist = idleAnchor = readDistanceCm();

  renderFace(FACE_WIDE);
}

void loop() {
  // cliff check first every loop - falling off the table is the worst case
  if (atEdge()) {
    edgeReflex();
    return;
  }

  if (millis() < startleUntil) return;           // hold the startle pose

  if (millis() - lastTick >= TICK_MS) {
    lastTick = millis();
    qTick();
  }

  if (millis() - lastSave >= SAVE_MS) {
    lastSave = millis();
    saveTable();
    Serial.printf("[save] step=%lu eps=%.3f  Q: D[%.1f %.1f %.1f] CL[%.1f %.1f %.1f] CF[%.1f %.1f %.1f] CR[%.1f %.1f %.1f]\n",
      stepCount, epsilon,
      Q[0][0],Q[0][1],Q[0][2], Q[1][0],Q[1][1],Q[1][2],
      Q[2][0],Q[2][1],Q[2][2], Q[3][0],Q[3][1],Q[3][2]);
  }
}
