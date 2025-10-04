#include "motor.h"

// Pins for Cytron MDD10A (DC motor driver)
const int dirPinA = 25; // Direction pin for motor A
const int pwmPinA = 33;  // PWM pin for motor A (connect to MDD10A PWM)

const int dirPinB = 13;  // Direction pin for motor B
const int pwmPinB = 26;  // PWM pin for motor B

// PWM configuration (ESP32 LEDC)
const int pwmFreq = 20000;      // 20 kHz
const int pwmChannelA = 0;
const int pwmChannelB = 1;
const int pwmResolution = 8;    // 8-bit resolution (0-255)

void motorSetup() {
  // Setup direction pins
  pinMode(dirPinA, OUTPUT);
  pinMode(dirPinB, OUTPUT);

  // Setup LEDC PWM channels
  ledcSetup(pwmChannelA, pwmFreq, pwmResolution);
  ledcAttachPin(pwmPinA, pwmChannelA);

  ledcSetup(pwmChannelB, pwmFreq, pwmResolution);
  ledcAttachPin(pwmPinB, pwmChannelB);

  // Ensure motors stopped initially
  digitalWrite(dirPinA, LOW);
  digitalWrite(dirPinB, LOW);
  ledcWrite(pwmChannelA, 0);
  ledcWrite(pwmChannelB, 0);
}

// Map PID output (0..255) to PWM duty (0..255)
int mapDoubleToPwm(double x, double in_min, double in_max) {
  if (x <= in_min) return 0;
  if (x >= in_max) return 255;
  return (int)((x - in_min) * 255.0 / (in_max - in_min));
}

void updateMotorControl(double leftCommand, double rightCommand) {
  // Left motor
  bool dirL = leftCommand >= 0;
  double absL = leftCommand >= 0 ? leftCommand : -leftCommand;
  if (absL > 255) absL = 255;
  int dutyL = mapDoubleToPwm(absL, 0.0, 255.0);
  digitalWrite(dirPinA, dirL ? HIGH : LOW);
  ledcWrite(pwmChannelA, dutyL);

  // Right motor
  bool dirR = rightCommand >= 0;
  double absR = rightCommand >= 0 ? rightCommand : -rightCommand;
  if (absR > 255) absR = 255;
  int dutyR = mapDoubleToPwm(absR, 0.0, 255.0);
  digitalWrite(dirPinB, dirR ? HIGH : LOW);
  ledcWrite(pwmChannelB, dutyR);
}
