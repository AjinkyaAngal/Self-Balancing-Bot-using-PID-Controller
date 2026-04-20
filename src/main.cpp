#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

// --- Global Variables & PID Parameters ---
float mpu_val = 0;
float maxFront = 0;
float maxBack = 0;

float p = 70;  
float d = 0;
float i = 0.1; 
float f = 1;

// --- Motor Pins & Logic Variables ---
long timer = 0;
int M11 = 3;
int M12 = 4;
int en1 = 10;
int M21 = 7;
int M22 = 8;
int en2 = 9;

float output = 0;
float pwm = 0;
float error = 0;
float adderr = 0;
float lastError = 0;
float derror = 0;
float falling = 0;
float tolerence = 0.3;

unsigned long timing = 0;
int correction = 0;
int dir = 0;
int prevDir = 0;

void setup() {
  Serial.begin(9600);
  
  pinMode(M11, OUTPUT);
  pinMode(M12, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(M21, OUTPUT);
  pinMode(M22, OUTPUT);
  pinMode(en2, OUTPUT);

  Wire.begin();

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  
  while (status != 0) {} // Halt if MPU connection fails
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(50); 
  mpu.calcOffsets(true, true);
  Serial.println("Done!\n");
}

void loop() {
  mpu.update();
  mpu_val = mpu.getAngleX();
  error = mpu_val;
  derror = error - lastError;
  falling = mpu.getGyroX();
  
  if (millis() - timing > 1000) {
    correction = 50;
  }
  
  // Calculate PID Output
  if (error > 0) {
    output = p * error + d * derror + i * adderr + f * falling + correction;
  } else {
    output = p * error + d * derror + i * adderr + f * falling + correction;
  }
  
  pwm = constrain(abs(output), 0, 255);
  
  // Motor Control Logic
  if (error > tolerence) {
    dir = 1;
    if(prevDir != dir){
      timing = millis();
    }
    digitalWrite(M11, HIGH);
    digitalWrite(M12, LOW);
    digitalWrite(M21, LOW);
    digitalWrite(M22, HIGH);
    analogWrite(en1, pwm);
    analogWrite(en2, pwm);
    
  } else if (error < -tolerence) {
    dir = -1;
    if(prevDir != dir){
      timing = millis();
    }
    digitalWrite(M11, LOW);
    digitalWrite(M12, HIGH);
    digitalWrite(M21, HIGH);
    digitalWrite(M22, LOW);
    analogWrite(en1, pwm);
    analogWrite(en2, pwm);
    
  } else {
    timing = millis();
    pwm = 0;
    dir = 0;
    analogWrite(en1, pwm);
    analogWrite(en2, pwm);
    adderr = 0;
    correction = 0;
  }
  
  lastError = error;

  // Print data every 50ms
  if (millis() - timer > 50) {  
    Serial.print(F("ANGLE     X: "));
    Serial.print(mpu.getAngleX());
    Serial.print("\tderror: ");
    Serial.print(d * derror);
    Serial.print("\terror: ");
    Serial.print(p * error);
    Serial.print("\tintegral: ");
    Serial.print(i * adderr);
    Serial.print("\tfalling: ");
    Serial.print(f * falling);
    Serial.print("\tcorrection: ");
    Serial.print(correction);
    Serial.print("\tpwm: ");
    Serial.print(pwm);
    Serial.println();
    timer = millis();
  }

  adderr += error;
  
  if (mpu_val > maxFront) {
    maxFront = mpu_val;
  }
  if (mpu_val < maxBack) {
    maxBack = mpu_val;
  }
  
  prevDir = dir;
}