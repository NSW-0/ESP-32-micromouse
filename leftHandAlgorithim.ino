#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <MPU6050.h>

// =====================
// PINS (ESP32)
// =====================
#define I2C_SDA 21
#define I2C_SCL 22

#define XSHUT_LEFT   13
#define XSHUT_FRONT  14
#define XSHUT_RIGHT  15

#define ADDR_LEFT   0x30
#define ADDR_FRONT  0x31
#define ADDR_RIGHT  0x32

// L298N -> ESP32
#define ENA 25
#define IN1 26
#define IN2 27

#define ENB 19
#define IN3 18
#define IN4 17

// Encoders
#define ENC_A_A 34
#define ENC_A_B 35
#define ENC_B_A 32
#define ENC_B_B 33

// =====================
// PWM (ESP32 LEDC v3.x)
// =====================
const int PWM_FREQ = 1000;
const int PWM_RES  = 8;   // 0..255

// =====================
// TUNE ZONE (keep your values)
// =====================

// Forward speed (SLOW)
int baseDuty = 55;
int trimA = 0;
int trimB = 1.6;
// Encoder & yaw corrections (gentle)
const float Kp_enc = 0.20f;
const int   MAX_CORR_ENC = 12;

const float Kp_yaw_straight = 0.9f;
const int   MAX_CORR_YAW_ST = 10;

// Wall thresholds
const int THRESH_FRONT_MM = 160;

// Side behavior
const int SIDE_MAX_MM     = 100;
const int THRESH_SIDE_MM  = 90;

// Timing
const unsigned long CONTROL_PERIOD_MS  = 20;   // motor update
const unsigned long FRONT_SENSE_MS     = 20;   // front check (faster than before)
const unsigned long SIDE_SENSE_MS      = 400;

const unsigned long STOP_MS            = 250;
const unsigned long POST_TURN_FWD_MS   = 2000;

// ✅ hard brake time (for best stopping)
const unsigned long BRAKE_MS           = 140;  // 100..200ms works well

// Turn tuning (UNCHANGED)
const float TURN_DEG_LEFT  = 76.0f;
const float TURN_SLOWDOWN  = 18.0f;
const int   TURN_DUTY_FAST = 55;
const int   TURN_DUTY_SLOW = 40;
const unsigned long TURN_TIMEOUT_MS = 2500;

const unsigned long BETWEEN_TURNS_MS = 90;

// =====================
// TYPES
// =====================
enum Decision { GO_FWD, TURN_L, TURN_R, TURN_U };
enum State    { RUNNING, STOPPING, TURNING, POST_TURN_FWD };

// =====================
// GLOBALS
// =====================
Adafruit_VL53L0X loxL, loxF, loxR;
MPU6050 mpu;

volatile long countA = 0;
volatile long countB = 0;

float yawDeg = 0.0f;
float gyroZ_bias_dps = 0.0f;
unsigned long lastIMUms = 0;

State state = RUNNING;
Decision pending = GO_FWD;

unsigned long tControl    = 0;
unsigned long tStop       = 0;
unsigned long tPost       = 0;

unsigned long tFrontSense = 0;
unsigned long tSideSense  = 0;

bool L_open_cache = false;
bool F_open_cache = true;
bool R_open_cache = false;

// =====================
// HELPERS
// =====================
int clampDuty(int x) {
  if (x < 0) return 0;
  if (x > 255) return 255;
  return x;
}
int clampSym(int x, int lim) {
  if (x >  lim) return  lim;
  if (x < -lim) return -lim;
  return x;
}

// =====================
// TOF INIT (3 sensors)
// =====================
bool beginWithRetries(Adafruit_VL53L0X &lox, uint8_t addr, int tries) {
  for (int i = 0; i < tries; i++) {
    if (lox.begin(addr, false, &Wire)) return true;
    delay(80);
  }
  return false;
}

bool bringUpAndSetAddress(Adafruit_VL53L0X &lox, int xshutPin, uint8_t newAddr) {
  digitalWrite(xshutPin, LOW);  delay(50);
  digitalWrite(xshutPin, HIGH); delay(150);

  if (!beginWithRetries(lox, 0x29, 6)) return false;

  lox.setAddress(newAddr);
  delay(20);

  if (!beginWithRetries(lox, newAddr, 3)) return false;
  return true;
}

int readMM_andStatus(Adafruit_VL53L0X &lox, uint8_t &statusOut) {
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m, false);
  statusOut = m.RangeStatus;
  if (m.RangeStatus != 0) return -1;
  return (int)m.RangeMilliMeter;
}

// =====================
// MPU (Yaw only)
// =====================
void calibrateGyroZ() {
  const int N = 600;
  long sum = 0;
  for (int i = 0; i < N; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
    sum += gz;
    delay(3);
  }
  float avg_gz = (float)sum / (float)N;
  gyroZ_bias_dps = avg_gz / 131.0f;
}

void resetYaw() {
  yawDeg = 0.0f;
  lastIMUms = millis();
}

void updateYaw() {
  unsigned long now = millis();
  float dt = (now - lastIMUms) / 1000.0f;
  lastIMUms = now;
  if (dt <= 0) return;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

  float gyroZ_dps = (gz / 131.0f) - gyroZ_bias_dps;
  yawDeg += gyroZ_dps * dt;
}

// =====================
// ENCODERS
// =====================
void IRAM_ATTR isrEncA_A() {
  int b = digitalRead(ENC_A_B);
  if (b == HIGH) countA++;
  else           countA--;
}
void IRAM_ATTR isrEncB_A() {
  int b = digitalRead(ENC_B_B);
  if (b == HIGH) countB++;
  else           countB--;
}

// =====================
// MOTORS
// =====================
void setForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // A forward
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);   // B forward
}

void setStopDirections() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// ✅ HARD BRAKE (best stopping): short the motor terminals via H-bridge
void brakeMotors() {
  // Brake mode: IN1=IN2=HIGH (motor A), IN3=IN4=HIGH (motor B)
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
}

void stopMotors() {
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
  setStopDirections();
}

// Smooth straight controller: encoders + yaw hold (kept as you had it)
void driveStraightController() {
  static long lastA = 0, lastB = 0;
  static float targetYawDeg = 0.0f;

  updateYaw();

  long a = countA;
  long b = countB;

  long dA = a - lastA;
  long dB = b - lastB;
  lastA = a;
  lastB = b;

  dA = abs(dA);
  dB = abs(dB);

  long errEnc = dA - dB;
  int corrEnc = clampSym((int)(Kp_enc * errEnc), MAX_CORR_ENC);

  float yawErr = yawDeg - targetYawDeg;
  int corrYaw = clampSym((int)(Kp_yaw_straight * yawErr), MAX_CORR_YAW_ST);

  int dutyA = baseDuty - corrEnc - corrYaw + trimA;
  int dutyB = baseDuty + corrEnc + corrYaw + trimB;

  setForward();
  ledcWrite(ENA, clampDuty(dutyA));
  ledcWrite(ENB, clampDuty(dutyB));
}

// =====================
// IN-PLACE TURNS (UNCHANGED)
// =====================
void turnLeft90_MPU_inPlace_WORKING() {
  stopMotors();
  delay(120);

  resetYaw();
  delay(10);

  // A forward, B backward
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);

  unsigned long t0 = millis();

  while (millis() - t0 < TURN_TIMEOUT_MS) {
    updateYaw();

    float remaining = TURN_DEG_LEFT - fabs(yawDeg);
    int duty = (remaining < TURN_SLOWDOWN) ? TURN_DUTY_SLOW : TURN_DUTY_FAST;

    ledcWrite(ENA, duty);
    ledcWrite(ENB, duty);

    if (fabs(yawDeg) >= TURN_DEG_LEFT) break;
    delay(5);
  }

  stopMotors();
  delay(120);
  resetYaw();
}

void turnRight90_FAKE() {
  turnLeft90_MPU_inPlace_WORKING();
  delay(BETWEEN_TURNS_MS);
  turnLeft90_MPU_inPlace_WORKING();
  delay(BETWEEN_TURNS_MS);
  turnLeft90_MPU_inPlace_WORKING();
}

void turnAround_FAKE() {
  turnLeft90_MPU_inPlace_WORKING();
  delay(BETWEEN_TURNS_MS);
  turnLeft90_MPU_inPlace_WORKING();
}

// =====================
// LEFT HAND DECISION
// =====================
Decision leftHandDecision(bool L_open, bool F_open, bool R_open) {
  if (L_open) return TURN_L;
  if (F_open) return GO_FWD;
  if (R_open) return TURN_R;
  return TURN_U;
}

// =====================
// SENSOR READERS
// =====================
void readFrontFlag(bool &F_open) {
  uint8_t stF = 0;
  int F = readMM_andStatus(loxF, stF);

  if (F < 0) F = 999; // invalid treated as far (open)
  F_open = (F >= THRESH_FRONT_MM);
}

void readSideFlags(bool &L_open, bool &R_open) {
  uint8_t stL=0, stR=0;
  int L = readMM_andStatus(loxL, stL);
  int R = readMM_andStatus(loxR, stR);

  if (L < 0) L = 999;
  if (R < 0) R = 999;

  if (L > SIDE_MAX_MM) L = SIDE_MAX_MM;
  if (R > SIDE_MAX_MM) R = SIDE_MAX_MM;

  L_open = (L >= THRESH_SIDE_MM);
  R_open = (R >= THRESH_SIDE_MM);
}

// =====================
// SETUP
// =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  // ToF power-up sequence
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(200);

  bool okL = bringUpAndSetAddress(loxL, XSHUT_LEFT,  ADDR_LEFT);
  bool okF = bringUpAndSetAddress(loxF, XSHUT_FRONT, ADDR_FRONT);
  bool okR = bringUpAndSetAddress(loxR, XSHUT_RIGHT, ADDR_RIGHT);
  if (!(okL && okF && okR)) {
    Serial.println("❌ TOF init failed");
    while (1) delay(1000);
  }

  Wire.setClock(400000);




  // MPU
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("❌ MPU6050 not found");
    while (1) delay(1000);
  }
  calibrateGyroZ();
  resetYaw();

  // Encoders
  pinMode(ENC_A_A, INPUT_PULLUP);
  pinMode(ENC_A_B, INPUT_PULLUP);
  pinMode(ENC_B_A, INPUT_PULLUP);
  pinMode(ENC_B_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A_A), isrEncA_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B_A), isrEncB_A, RISING);

  // Motors
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  ledcAttach(ENA, PWM_FREQ, PWM_RES);
  ledcAttach(ENB, PWM_FREQ, PWM_RES);

  stopMotors();

  // Timers
  tControl    = millis();
  tFrontSense = millis();
  tSideSense  = millis();

  // Prime caches
  readFrontFlag(F_open_cache);
  readSideFlags(L_open_cache, R_open_cache);
}

// =====================
// LOOP
// =====================
void loop() {
  unsigned long now = millis();

  if (state == RUNNING) {

    // ✅ Option A: emergency front check BEFORE driving (every control tick)
    if (now - tControl >= CONTROL_PERIOD_MS) {
      tControl = now;

      // front check right now (fast reaction)
      readFrontFlag(F_open_cache);
      if (!F_open_cache) {
        // ✅ best stop: BRAKE immediately
        brakeMotors();
        pending = leftHandDecision(L_open_cache, F_open_cache, R_open_cache);
        tStop = now;
        state = STOPPING;
        return;
      }

      // only drive if safe
      driveStraightController();
    }

    // extra front check (still useful)
    if (now - tFrontSense >= FRONT_SENSE_MS) {
      tFrontSense = now;
      readFrontFlag(F_open_cache);

      if (!F_open_cache) {
        brakeMotors();
        pending = leftHandDecision(L_open_cache, F_open_cache, R_open_cache);
        tStop = now;
        state = STOPPING;
        return;
      }
    }

    // side check slow
    if (now - tSideSense >= SIDE_SENSE_MS) {
      tSideSense = now;
      readSideFlags(L_open_cache, R_open_cache);

      if (L_open_cache) {
        // stopping for turn: brake too
        brakeMotors();
        pending = leftHandDecision(L_open_cache, F_open_cache, R_open_cache);
        tStop = now;
        state = STOPPING;
        return;
      }
    }

    return;
  }

  if (state == STOPPING) {
    // ✅ hold brake for a short time to kill momentum, then coast stop
    if (now - tStop <= BRAKE_MS) brakeMotors();
    else                         stopMotors();

    if (now - tStop >= STOP_MS) state = TURNING;
    return;
  }

  if (state == TURNING) {
    if (pending == TURN_L) {
      turnLeft90_MPU_inPlace_WORKING();
    } else if (pending == TURN_R) {
      turnRight90_FAKE();
    } else if (pending == TURN_U) {
      turnAround_FAKE();
    } else {
      // GO_FWD -> do nothing
    }

    tPost       = millis();
    tControl    = millis();
    tFrontSense = millis();
    tSideSense  = millis();

    state = POST_TURN_FWD;
    return;
  }

  if (state == POST_TURN_FWD) {
    if (now - tControl >= CONTROL_PERIOD_MS) {
      tControl = now;

      // ✅ also emergency stop in post-forward
      readFrontFlag(F_open_cache);
      if (!F_open_cache) {
        brakeMotors();
        pending = TURN_L; // safest behavior; you can keep your decision function if you want
        tStop = now;
        state = STOPPING;
        return;
      }

      driveStraightController();
    }

    if (now - tPost >= POST_TURN_FWD_MS) {
      tFrontSense = now;
      tSideSense  = now;
      state = RUNNING;
    }
    return;
  }
}
