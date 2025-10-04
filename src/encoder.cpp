#include "encoder.h"

// Default pins (change if needed)
const int ENC_L_A = 35; // Left encoder A
const int ENC_L_B = 36; // Left encoder B
const int ENC_R_A = 34; // Right encoder A
const int ENC_R_B = 39; // Right encoder B

// Pulses per revolution (set according to your encoder)
const int PPR = 1024;

volatile long leftCount = 0;
volatile long rightCount = 0;

// For speed calculation
static long lastLeftCount = 0;
static long lastRightCount = 0;
static unsigned long lastSpeedCalc = 0;
static double leftCps = 0.0;
static double rightCps = 0.0;

// Simple quadrature ISR handlers
void IRAM_ATTR handleLeftA() {
  bool b = digitalRead(ENC_L_B);
  if (b) leftCount++; else leftCount--;
}

void IRAM_ATTR handleRightA() {
  bool b = digitalRead(ENC_R_B);
  if (b) rightCount++; else rightCount--;
}

void encoderSetup() {
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), handleLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), handleRightA, CHANGE);

  lastSpeedCalc = millis();
}

void encoderUpdate() {
  unsigned long now = millis();
  unsigned long dt = now - lastSpeedCalc;
  if (dt >= 100) { // update every 100 ms
    long l = leftCount;
    long r = rightCount;
    leftCps = (double)(l - lastLeftCount) / (dt / 1000.0);
    rightCps = (double)(r - lastRightCount) / (dt / 1000.0);
    lastLeftCount = l;
    lastRightCount = r;
    lastSpeedCalc = now;
  }
}

long getLeftCount() { return leftCount; }
long getRightCount() { return rightCount; }

double getLeftSpeedCps() { return leftCps; }
double getRightSpeedCps() { return rightCps; }
