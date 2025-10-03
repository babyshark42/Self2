#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

// Quadrature encoders (left and right)
// Configure pins in encoder.cpp

// Initialize encoder hardware
void encoderSetup();

// Call periodically to update computed speeds (counts per second)
void encoderUpdate();

// Read raw counts
long getLeftCount();
long getRightCount();

// Get speed in counts per second (averaged over last interval)
double getLeftSpeedCps();
double getRightSpeedCps();

#endif // ENCODER_H
