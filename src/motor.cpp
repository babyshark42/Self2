#include "motor.h"

// ===== Pin mapping for Cytron MDD10A =====
const int dirPinA = 25;  // Left  DIR
const int pwmPinA = 33;  // Left  PWM
const int dirPinB = 13;  // Right DIR
const int pwmPinB = 26;  // Right PWM

// ===== PWM config (LEDC on ESP32) =====
const int pwmFreq       = 20000; // 20 kHz (เงียบหู)
const int pwmChannelA   = 0;
const int pwmChannelB   = 1;
const int pwmResolution = 8;     // 0..255

// ===== Direction inversion (แก้ทิศล้อ) =====
// ส่วนใหญ่ “ล้อขวา” ต้องกลับทิศ หากใส่แล้วไปผิดทิศให้สลับ true/false เอา
static const bool INVERT_LEFT  = true;
static const bool INVERT_RIGHT = false;

// ===== Helpers =====
static inline int clamp255(double v) {
  if (v < 0)   return 0;
  if (v > 255) return 255;
  return (int)v;
}

int mapDoubleToPwm(double x, double in_min, double in_max) {
  if (in_max == in_min) return 0;
  double t = (x - in_min) * 255.0 / (in_max - in_min);
  return clamp255(t);
}

// ===== Public APIs =====
void motorSetup() {
  pinMode(dirPinA, OUTPUT);
  pinMode(pwmPinA, OUTPUT);
  pinMode(dirPinB, OUTPUT);
  pinMode(pwmPinB, OUTPUT);

  ledcSetup(pwmChannelA, pwmFreq, pwmResolution);
  ledcSetup(pwmChannelB, pwmFreq, pwmResolution);
  ledcAttachPin(pwmPinA, pwmChannelA);
  ledcAttachPin(pwmPinB, pwmChannelB);

  // มอเตอร์หยุด
  digitalWrite(dirPinA, LOW);
  digitalWrite(dirPinB, LOW);
  ledcWrite(pwmChannelA, 0);
  ledcWrite(pwmChannelB, 0);
}

// คำสั่งช่วง -255 .. +255 (ลบ=ถอยหลัง, บวก=เดินหน้า)
void updateMotorControl(double leftCommand, double rightCommand) {
  // ----- Left motor -----
  bool dirL = (leftCommand >= 0.0);
  double absL = dirL ? leftCommand : -leftCommand;
  int dutyL = mapDoubleToPwm(absL, 0.0, 255.0);
  digitalWrite(dirPinA, (dirL ^ INVERT_LEFT) ? HIGH : LOW);
  ledcWrite(pwmChannelA, dutyL);

  // ----- Right motor -----
  bool dirR = (rightCommand >= 0.0);
  double absR = dirR ? rightCommand : -rightCommand;
  int dutyR = mapDoubleToPwm(absR, 0.0, 255.0);
  digitalWrite(dirPinB, (dirR ^ INVERT_RIGHT) ? HIGH : LOW);
  ledcWrite(pwmChannelB, dutyR);
}
