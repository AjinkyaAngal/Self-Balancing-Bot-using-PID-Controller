// ============================================================
//  Self-Balancing Bot — PlatformIO (main.cpp)
//  Hardware : Arduino Uno + MPU6050 + L298N
//
//  Pin Map:
//    MPU6050  VCC → 3.3V | GND → GND | SCL → SCL | SDA → SDA
//    L298N    ENA → 10   | IN1 → 9   | IN2 → 8
//             IN3 → 7   | IN4 → 6   | ENB → 5
//             5V  → Vin | GND → GND
//
//  Tuning Order:
//    1. Hold bot UPRIGHT & STILL → upload → calibration + auto-setpoint runs
//    2. Check Serial Monitor: "Auto-setpoint set to: X.XX"
//    3. Find MIN_PWM: set Kp=255, Ki=0, Kd=0 → find lowest PWM
//       that moves wheels on ground → set as MIN_PWM
//    4. Kp: start at 5, increase until bot oscillates
//    5. Kd: increase until oscillations damp
//    6. Ki: add last, start at 0.5, only if steady lean persists
//
//  NOTE: Hold bot STRAIGHT AND STILL during first ~3 seconds
//        of every boot (calibration + angle averaging period)
// ============================================================

#include <Arduino.h>
#include <Wire.h>

// ─── MPU6050 ──────────────────────────────────────────────────
#define MPU_ADDR 0x68

// ─── L298N Motor Pins ─────────────────────────────────────────
#define ENA 10
#define IN1  9
#define IN2  8
#define IN3  7
#define IN4  6
#define ENB  5

// ─── PID Constants — TUNE THESE ───────────────────────────────
double Kp = 70.0;
double Ki = 0.0;
double Kd = 0.0;

// ─── SETPOINT: DO NOT HARDCODE THIS ANYMORE ───────────────────
// This is now automatically determined at boot by averaging
// 300 accelerometer samples while the bot is held upright.
// Just hold the bot straight during calibration and it sets itself.
double SETPOINT = 0.0;   // gets overwritten in setup()

// ─── Motor Config ─────────────────────────────────────────────
const int MIN_PWM = 60;
const int MAX_PWM = 255;

// Bot has fallen if angle deviates beyond this from setpoint
const double FALL_ANGLE = 40.0;

// ─── Filter Config ────────────────────────────────────────────
// 0.96 = 96% gyro, 4% accel. Reduces drift vs 0.98.
// Do NOT go below 0.95 — motor vibration corrupts accel.
const double ALPHA = 0.96;

// Gyro values below this (°/s) are noise → treated as zero
const double GYRO_DEADBAND = 0.5;

// ─── Calibration Config ───────────────────────────────────────
const int GYRO_CALIB_SAMPLES = 500;  // ~1.5 seconds
const int ACC_AVG_SAMPLES    = 300;  // ~0.9 seconds

// ─── Runtime State ────────────────────────────────────────────
double currentAngle = 0.0;
double errSum       = 0.0;
double lastErr      = 0.0;
float  gyroX_offset = 0.0;
unsigned long lastTime = 0;

int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;


// ═════════════════════════════════════════════════════════════
void initMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);   // Wake up
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x00);   // Gyro ±250°/s
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x00);   // Accel ±2g
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A); Wire.write(0x03);   // DLPF ~44Hz
  Wire.endTransmission(true);
}


// ═════════════════════════════════════════════════════════════
void readMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  accX  = (Wire.read() << 8) | Wire.read();
  accY  = (Wire.read() << 8) | Wire.read();
  accZ  = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();   // Skip temperature
  gyroX = (Wire.read() << 8) | Wire.read();
  gyroY = (Wire.read() << 8) | Wire.read();
  gyroZ = (Wire.read() << 8) | Wire.read();
}


// ═════════════════════════════════════════════════════════════
//  GYRO CALIBRATION — keep bot STILL
// ═════════════════════════════════════════════════════════════
void calibrateGyro() {
  Serial.println(">> [1/2] Gyro calibration... keep STILL");
  long sum = 0;
  for (int i = 0; i < GYRO_CALIB_SAMPLES; i++) {
    readMPU6050();
    sum += gyroX;
    delay(3);
  }
  gyroX_offset = (float)sum / GYRO_CALIB_SAMPLES;
  Serial.print(">>  Gyro offset: ");
  Serial.println(gyroX_offset, 4);
}


// ═════════════════════════════════════════════════════════════
//  ACCELEROMETER ANGLE AVERAGING — THE KEY FIX
//
//  OLD: currentAngle = atan2(accY, accZ) from ONE sample → noisy
//  NEW: Average 300 samples → stable, consistent starting angle
//
//  Also auto-sets SETPOINT = averaged boot angle.
//  No more manually chasing a hardcoded number every boot.
// ═════════════════════════════════════════════════════════════
void calibrateStartAngle() {
  Serial.println(">> [2/2] Averaging start angle... keep STILL & UPRIGHT");
  double angleSum = 0.0;

  for (int i = 0; i < ACC_AVG_SAMPLES; i++) {
    readMPU6050();
    // If upright reads ~±90°, swap accY → accX in this line AND in loop()
    angleSum += atan2((double)accY, (double)accZ) * 180.0 / PI;
    delay(3);
  }

  currentAngle = angleSum / ACC_AVG_SAMPLES;
  SETPOINT     = currentAngle;   // auto-set, no manual tuning needed

  Serial.print(">>  Start angle:   "); Serial.println(currentAngle, 4);
  Serial.print(">>  Auto-setpoint: "); Serial.println(SETPOINT, 4);
  Serial.println(">> Calibration complete.");
}


// ═════════════════════════════════════════════════════════════
//  PID — Anti-windup clamped in output-space
// ═════════════════════════════════════════════════════════════
double computePID(double angle, double dt) {
  double error = SETPOINT - angle;

  if (Ki > 0.0) {
    errSum += error * dt;
    errSum  = constrain(errSum, -MAX_PWM / Ki, MAX_PWM / Ki);
  }

  double dErr = (error - lastErr) / dt;
  lastErr = error;

  double output = (Kp * error) + (Ki * errSum) + (Kd * dErr);
  return constrain(output, -MAX_PWM, MAX_PWM);
}


// ═════════════════════════════════════════════════════════════
//  MOTOR DRIVER
// ═════════════════════════════════════════════════════════════
void driveMotors(double output) {
  if (abs(currentAngle - SETPOINT) > FALL_ANGLE) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);    analogWrite(ENB, 0);
    return;
  }

  int spd = (int)abs(output);

  if (spd < MIN_PWM) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);    analogWrite(ENB, 0);
    return;
  }

  if (output > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  }

  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}


// ═════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════
void setup() {
  // MUST be 115200 — at 9600 baud, Serial.print() alone takes
  // longer than the 5ms loop delay, corrupting dt calculations.
  Serial.begin(115200);
  Wire.begin();

  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  analogWrite(ENA, 0);  analogWrite(ENB, 0);

  initMPU6050();
  delay(500);

  calibrateGyro();        // Step 1: gyro bias (~1.5s)
  calibrateStartAngle();  // Step 2: stable angle + auto-setpoint (~0.9s)

  lastTime = millis();

  Serial.println("==============================================");
  Serial.print  ("  SETPOINT auto-set to: ");
  Serial.println(SETPOINT, 2);
  Serial.println("  Bot ready. Open Serial Plotter to monitor.");
  Serial.println("==============================================");
}


// ═════════════════════════════════════════════════════════════
//  LOOP — ~200Hz
// ═════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();
  double dt = (now - lastTime) / 1000.0;
  lastTime = now;

  if (dt <= 0.0 || dt > 0.1) return;

  readMPU6050();

  // If upright reads ~±90°, swap accY → accX in BOTH lines below
  double accAngle = atan2((double)accY, (double)accZ) * 180.0 / PI;
  double gyroRate = (double)(gyroX - gyroX_offset) / 131.0;

  if (abs(gyroRate) < GYRO_DEADBAND) gyroRate = 0.0;

  currentAngle = ALPHA * (currentAngle + gyroRate * dt)
               + (1.0 - ALPHA) * accAngle;

  double output = computePID(currentAngle, dt);
  driveMotors(output);

  // Serial Plotter compatible — Tools → Serial Plotter
  Serial.print("Angle:");     Serial.print(currentAngle, 2);
  Serial.print(",Setpoint:"); Serial.print(SETPOINT, 2);
  Serial.print(",Output:");   Serial.println(output, 2);

  delay(5);
}