// ===================== main.cpp (stable balance version) =====================
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include "encoder.h"
#include "motor.h"

enum class DriveMode { OpenLoopPWM, VelocityPI, Balance };
static DriveMode mode = DriveMode::OpenLoopPWM;

// ===================== PWM =====================
static int cmdMag = 120;
static int cmdL = 120;
static int cmdR = 120;

static inline double clamp(double x, double lo, double hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}
static int applyDeadbandSigned(double cmd, int startPwm) {
  double m = fabs(cmd);
  if (m < 1.0) return 0;
  double out = startPwm + (255.0 - startPwm) * (m / 255.0);
  int duty = (int)(out + 0.5);
  return (cmd >= 0) ? duty : -duty;
}

// ===================== Velocity PI =====================
static double KpL = 0.08, KiL = 0.45;
static double KpR = 0.08, KiR = 0.45;
static double iL = 0.0, iR = 0.0;
static const int PWM_MAX = 255;
static double kV_L = 2.55, kV_R = 2.80;
static const int START_PWM_L = 24;
static const int START_PWM_R = 28;
static double targetRpmL = 60.0;
static double targetRpmR = 60.0;
static int gDutyL = 0, gDutyR = 0;
static bool gSatL = false, gSatR = false;
static uint32_t tCtrl = 0;
static const uint32_t CTRL_DT_MS = 10;

static void applyCommandPWM() {
  cmdMag = constrain(cmdMag, 0, 255);
  cmdL = constrain(cmdL, -255, 255);
  cmdR = constrain(cmdR, -255, 255);
  gDutyL = cmdL; gDutyR = cmdR;
  updateMotorControl((double)cmdL, (double)cmdR);
}

static void speedControlStep() {
  const double CPR = 4.0 * 1024.0;
  const double Ts = CTRL_DT_MS / 1000.0;

  double Lrpm = getLeftSpeedCps() * 60.0 / CPR;
  double Rrpm = getRightSpeedCps() * 60.0 / CPR;

  double uL = kV_L * targetRpmL;
  double uR = kV_R * targetRpmR;

  double eL = targetRpmL - Lrpm;
  double eR = targetRpmR - Rrpm;
  uL += KpL * eL + iL;
  uR += KpR * eR + iR;

  gSatL = (uL > PWM_MAX) || (uL < -PWM_MAX);
  gSatR = (uR > PWM_MAX) || (uR < -PWM_MAX);
  uL = clamp(uL, -PWM_MAX, PWM_MAX);
  uR = clamp(uR, -PWM_MAX, PWM_MAX);

  double iLnext = iL + KiL * Ts * eL;
  double iRnext = iR + KiR * Ts * eR;
  double uLnext = kV_L * targetRpmL + KpL * eL + iLnext;
  double uRnext = kV_R * targetRpmR + KpR * eR + iRnext;
  if (!gSatL || (uL == PWM_MAX && uLnext < uL) || (uL == -PWM_MAX && uLnext > uL)) iL = iLnext;
  if (!gSatR || (uR == PWM_MAX && uRnext < uR) || (uR == -PWM_MAX && uRnext > uR)) iR = iRnext;

  gDutyL = applyDeadbandSigned(uL, START_PWM_L);
  gDutyR = applyDeadbandSigned(uR, START_PWM_R);
  updateMotorControl(gDutyL, gDutyR);
}

// ===================== IMU / Balance =====================
MPU6050 imu(Wire);
static bool imuReady = false;
static double theta = 0.0;
static double thetaDot = 0.0;
static double thetaDotFilt = 0.0;
static double thetaOffset = 0.0;
static uint32_t tIMU = 0;

static double KpB = 9.0;     // ↓ ลดแรงพยายาม
static double KdB = 1.8;     // ↑ เพิ่ม damping
static double KiB = 0.00;
static double KvB = 0.015;
static double iB = 0.0;

static double thetaRef = 0.0;
static double turnCmd = 0.0;
static const double MAX_ABS_ANGLE = 17.0;

void imuUpdate() {
  imu.update();
  double gyroX = imu.getGyroX();
  thetaDotFilt = 0.8 * thetaDotFilt + 0.2 * gyroX;   // low-pass filter
  theta = imu.getAngleX() - thetaOffset;
  thetaDot = thetaDotFilt;
}

void balanceControlStep(uint32_t dt_us) {
  double thetaEff = clamp(theta, -15.0, 15.0);
  if (fabs(theta) > MAX_ABS_ANGLE) {
    iB = 0;
    updateMotorControl(0, 0);
    return;
  }

  double v_cps = 0.5 * (getLeftSpeedCps() + getRightSpeedCps());
  double e = thetaRef - thetaEff;
  double Ts = dt_us / 1e6;

  double u = KpB * e + KdB * (-thetaDot) + KvB * (-v_cps) + iB;
  double uC = clamp(u, -180, 180);   // limit PWM ±180

  double iNext = iB + KiB * Ts * e;
  if (fabs(u) < 180) iB = iNext;

  double uL = uC - turnCmd;
  double uR = uC + turnCmd;

  int dutyL = applyDeadbandSigned(uL, START_PWM_L);
  int dutyR = applyDeadbandSigned(uR, START_PWM_R);
  updateMotorControl(dutyL, dutyR);

  gDutyL = dutyL; gDutyR = dutyR;
  gSatL = (fabs(uL) >= 180);
  gSatR = (fabs(uR) >= 180);
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  encoderSetup();
  motorSetup();

  // I2C setup (ลดความถี่เพื่อความเสถียร)
  Wire.begin();
  Wire.setClock(100000);
  imuReady = (imu.begin() == 0);

  if (imuReady) {
    Serial.println(F("MPU6050 OK, keep robot still for calibration..."));
    imu.calcGyroOffsets();
    delay(300);

    double sum = 0;
    for (int i = 0; i < 300; i++) {
      imu.update();
      sum += imu.getAngleX();
      delay(2);
    }
    thetaOffset = sum / 300.0;
    theta = 0.0; thetaDot = 0.0; thetaDotFilt = 0.0;
    Serial.print(F("Pitch offset = ")); Serial.println(thetaOffset, 3);
  } else {
    Serial.println(F("MPU6050 FAILED — disable balance."));
  }

  mode = DriveMode::OpenLoopPWM;
  cmdL = cmdMag; cmdR = cmdMag;
  applyCommandPWM();
  tCtrl = millis();

  Serial.println(F("Boot OK. Mode = PWM ('v' for VEL, 'x' for BAL)."));
}

// ===================== LOOP =====================
void loop() {
  encoderUpdate();
  if (imuReady) imuUpdate();

  // Velocity PI
  if (mode == DriveMode::VelocityPI && millis() - tCtrl >= CTRL_DT_MS) {
    tCtrl += CTRL_DT_MS;
    speedControlStep();
  }

  // Balance 1 kHz
  static uint32_t tBal = micros();
  uint32_t now_us = micros();
  if (mode == DriveMode::Balance && now_us - tBal >= 1000) {
    uint32_t dt = now_us - tBal;
    tBal = now_us;
    balanceControlStep(dt);
  }

  // Serial command
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'x' || c == 'X') {
      if (!imuReady) Serial.println(F("IMU not ready."));
      else {
        mode = (mode == DriveMode::Balance) ? DriveMode::VelocityPI : DriveMode::Balance;
        iB = 0; thetaRef = 0; turnCmd = 0;
        updateMotorControl(0, 0);
        Serial.print(F("Mode = "));
        Serial.println(mode == DriveMode::Balance ? "BAL" : "VEL");
      }
    }
  }
}
