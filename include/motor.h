#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

// Cytron MDD10A DC motor interface (ESP32)
// DIR pin controls direction (HIGH/LOW)
// PWM pin controls speed (0..255 mapped to LEDC duty)

// Motor A (left)
extern const int dirPinA;
extern const int pwmPinA;

// Motor B (right)
extern const int dirPinB;
extern const int pwmPinB;

// PWM configuration
extern const int pwmFreq;     // PWM frequency
extern const int pwmChannelA; // LEDC channel for motor A
extern const int pwmChannelB; // LEDC channel for motor B
extern const int pwmResolution; // PWM resolution (bits)

// Function prototypes
void motorSetup();
// Update motor control with separate commands for left and right (-255..255)
void updateMotorControl(double leftCommand, double rightCommand);
int mapDoubleToPwm(double x, double in_min, double in_max);

#endif  // MOTOR_H
