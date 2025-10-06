// src/main.cpp
#include <Arduino.h>
#include "encoder.h"
#include "motor.h"

// ====== โหมดการขับ ======
enum class DriveMode { OpenLoopPWM, VelocityPI };
static DriveMode mode = DriveMode::OpenLoopPWM;

// ====== ค่าทดสอบโหมด PWM ตรง ======
static int cmdMag = 120;   // 0..255
static int cmdL   = 120;   // -255..255
static int cmdR   = 120;

// ====== พิมพ์สถานะ ======
static unsigned long tPrint = 0;
static const unsigned long PRINT_PERIOD = 500;  // ms

// ====== Velocity PI control (ต่อข้าง) ======
static double KpL = 0.03, KiL = 0.20;
static double KpR = 0.03, KiR = 0.20;
static double iL = 0.0,  iR = 0.0;            // integral terms
static const int PWM_MAX = 255;

// เดดโซนต่อข้าง (ใส่ค่าที่วัดจริง)
static const int START_PWM_L = 24;
static const int START_PWM_R = 32;

// เป้าหมายความเร็ว (RPM) เริ่มต้น
static double targetRpmL = 60.0;
static double targetRpmR = 60.0;

// จังหวะคุม 100 Hz
static uint32_t tCtrl = 0;
static const uint32_t CTRL_DT_MS = 10;

// ====== Helpers ======
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

static void applyCommandPWM() {
  cmdMag = constrain(cmdMag, 0, 255);
  cmdL   = constrain(cmdL,  -255, 255);
  cmdR   = constrain(cmdR,  -255, 255);
  updateMotorControl((double)cmdL, (double)cmdR);
}

static void speedControlStep() {
  // ใช้ CPR = 4*PPR (ของคุณ PPR=1024 → CPR=4096)
  const double CPR = 4.0 * 1024.0;

  // 1) วัดความเร็ว (cps → rpm)
  double Lrpm = getLeftSpeedCps()  * 60.0 / CPR;
  double Rrpm = getRightSpeedCps() * 60.0 / CPR;

  // 2) error
  double eL = targetRpmL - Lrpm;
  double eR = targetRpmR - Rrpm;

  // 3) PI
  const double Ts = CTRL_DT_MS / 1000.0;
  double uL = KpL * eL + iL;
  double uR = KpR * eR + iR;

  // 4) pre-clamp
  uL = clamp(uL, -PWM_MAX, PWM_MAX);
  uR = clamp(uR, -PWM_MAX, PWM_MAX);

  // 5) anti-windup แบบง่าย
  double iLnext = iL + KiL * Ts * eL;
  double iRnext = iR + KiR * Ts * eR;
  double uLnext = KpL * eL + iLnext;
  double uRnext = KpR * eR + iRnext;
  if (fabs(uL) < PWM_MAX || (uL == PWM_MAX && uLnext < uL) || (uL == -PWM_MAX && uLnext > uL)) iL = iLnext;
  if (fabs(uR) < PWM_MAX || (uR == PWM_MAX && uRnext < uR) || (uR == -PWM_MAX && uRnext > uR)) iR = iRnext;

  // 6) deadband + สั่งมอเตอร์
  int dutyL = applyDeadbandSigned(uL, START_PWM_L);
  int dutyR = applyDeadbandSigned(uR, START_PWM_R);
  updateMotorControl(dutyL, dutyR);
}

static void printHelp() {
  Serial.println();
  Serial.println(F("=== Motor Test ==="));
  Serial.println(F("  Modes:  p = PWM (open-loop),  v = Velocity PI (closed-loop)"));
  Serial.println(F("  f : forward  | b : backward  | l : turn left  | r : turn right  | s : stop"));
  Serial.println(F("  ]/+ : increase magnitude or target RPM (+10)"));
  Serial.println(F("  [/− : decrease magnitude or target RPM (−10)"));
  Serial.println(F("  h   : show this help"));
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  encoderSetup();
  motorSetup();

  // เริ่มที่โหมด PWM test
  mode = DriveMode::OpenLoopPWM;
  cmdL = cmdMag; cmdR = cmdMag;
  applyCommandPWM();

  tCtrl = millis();
  printHelp();
  Serial.println(F("Boot OK. Mode = PWM (open-loop). Press 'v' for closed-loop velocity."));
}

void loop() {
  encoderUpdate();

  // คุมความเร็วเมื่ออยู่โหมด PI
  uint32_t now = millis();
  if (mode == DriveMode::VelocityPI && (now - tCtrl) >= CTRL_DT_MS) {
    tCtrl += CTRL_DT_MS;
    speedControlStep();
  }

  // พิมพ์สถานะทุก 500 ms
  extern volatile uint32_t leftA_edges, rightA_edges;
  if (now - tPrint >= PRINT_PERIOD) {
    tPrint = now;

    const double CPR = 4.0 * 1024.0;
    double Lrpm = getLeftSpeedCps()  * 60.0 / CPR;
    double Rrpm = getRightSpeedCps() * 60.0 / CPR;

    Serial.print(F("Mode=")); Serial.print(mode == DriveMode::OpenLoopPWM ? "PWM" : "VEL");
    Serial.print(F(" | Lcnt="));  Serial.print(getLeftCount());
    Serial.print(F(", Rcnt="));   Serial.print(getRightCount());
    Serial.print(F(" | Lcps="));  Serial.print(getLeftSpeedCps(), 1);
    Serial.print(F(", Rcps="));   Serial.print(getRightSpeedCps(), 1);
    Serial.print(F(" | Lrpm="));  Serial.print(Lrpm,1);
    Serial.print(F(", Rrpm="));   Serial.print(Rrpm,1);
    if (mode == DriveMode::VelocityPI) {
      Serial.print(F(" | TrpmL=")); Serial.print(targetRpmL,0);
      Serial.print(F(", TrpmR="));  Serial.print(targetRpmR,0);
    } else {
      Serial.print(F(" | cmdL=")); Serial.print(cmdL);
      Serial.print(F(", cmdR="));  Serial.print(cmdR);
    }
    Serial.print(F(" | LA_edges=")); Serial.print(leftA_edges);
    Serial.print(F(", RA_edges="));  Serial.println(rightA_edges);
  }

  // อ่านคีย์บอร์ดจาก Serial Monitor (No line ending)
  if (Serial.available()) {
    char c = Serial.read();

    // สลับโหมด
    if (c == 'p' || c == 'P') {
      mode = DriveMode::OpenLoopPWM;
      iL = iR = 0;          // เคลียร์อินทิกรัล
      Serial.println(F("Mode = PWM (open-loop)"));
    } else if (c == 'v' || c == 'V') {
      mode = DriveMode::VelocityPI;
      iL = iR = 0;
      Serial.println(F("Mode = Velocity PI (closed-loop)"));
    }

    // คำสั่งตามโหมด
    switch (c) {
      // ------- คำสั่งทิศทางร่วมกัน --------
      case 'f': case 'F':
        if (mode == DriveMode::OpenLoopPWM) { cmdL = +cmdMag; cmdR = +cmdMag; applyCommandPWM(); }
        else { targetRpmL = targetRpmR = +60; }
        break;

      case 'b': case 'B':
        if (mode == DriveMode::OpenLoopPWM) { cmdL = -cmdMag; cmdR = -cmdMag; applyCommandPWM(); }
        else { targetRpmL = targetRpmR = -60; }
        break;

      case 'l': case 'L':
        if (mode == DriveMode::OpenLoopPWM) { cmdL = -cmdMag; cmdR = +cmdMag; applyCommandPWM(); }
        else { targetRpmL = -40; targetRpmR = +40; }
        break;

      case 'r': case 'R':
        if (mode == DriveMode::OpenLoopPWM) { cmdL = +cmdMag; cmdR = -cmdMag; applyCommandPWM(); }
        else { targetRpmL = +40; targetRpmR = -40; }
        break;

      case 's': case 'S':
        if (mode == DriveMode::OpenLoopPWM) { cmdL = 0; cmdR = 0; applyCommandPWM(); }
        else { targetRpmL = targetRpmR = 0; iL = iR = 0; }
        break;

      // ------- ปรับความแรง/ความเร็ว -------
      case '+': case ']':
        if (mode == DriveMode::OpenLoopPWM) {
          cmdMag = min(255, cmdMag + 10);
          if (cmdL > 0) cmdL = +cmdMag;
          if (cmdL < 0) cmdL = -cmdMag;
          if (cmdR > 0) cmdR = +cmdMag;
          if (cmdR < 0) cmdR = -cmdMag;
          applyCommandPWM();
          Serial.print(F("Magnitude = ")); Serial.println(cmdMag);
        } else {
          targetRpmL += 10; targetRpmR += 10;
          Serial.print(F("Target RPM = ")); Serial.println(targetRpmL);
        }
        break;

      case '-': case '[':
        if (mode == DriveMode::OpenLoopPWM) {
          cmdMag = max(0, cmdMag - 10);
          if (cmdL > 0) cmdL = +cmdMag;
          if (cmdL < 0) cmdL = -cmdMag;
          if (cmdR > 0) cmdR = +cmdMag;
          if (cmdR < 0) cmdR = -cmdMag;
          applyCommandPWM();
          Serial.print(F("Magnitude = ")); Serial.println(cmdMag);
        } else {
          targetRpmL -= 10; targetRpmR -= 10;
          Serial.print(F("Target RPM = ")); Serial.println(targetRpmL);
        }
        break;

      case 'h': case 'H':
        printHelp();
        break;

      default: break;
    }
  }
}
