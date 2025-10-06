#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
extern volatile uint32_t leftA_edges;
extern volatile uint32_t rightA_edges;
// Quadrature encoders (left and right)
// Configure pins in encoder.cpp
double cpsToRpm(double cps);
// Initialize encoder hardware
void encoderSetup();

// Print encoder speeds to Serial
void printEncoderSpeeds();

// Call periodically to update computed speeds (counts per second)
void encoderUpdate();

// Read raw counts
long getLeftCount();
long getRightCount();

// Get speed in counts per second (averaged over last interval)
double getLeftSpeedCps();
double getRightSpeedCps();

#endif // ENCODER_H
