#include "encoder.h"

// ===== Pins (ESP32 note: 34–39 have NO internal pull-ups) =====
const int ENC_L_A = 35;
const int ENC_L_B = 36;
const int ENC_R_A = 34;
const int ENC_R_B = 39;

// เพิ่มตัวแปรนับขอบ A ของซ้าย/ขวา (ประกาศตัวจริงในไฟล์นี้)
volatile uint32_t leftA_edges  = 0;
volatile uint32_t rightA_edges = 0;
// ===== Encoder config =====
const int PPR = 1024;   // pulses per revolution (per channel)
static const double CPR = 4.0 * PPR;  // A + B + rising/falling
double cpsToRpm(double cps) {
  return cps * 60.0 / CPR;
}

// กลับทิศนับ (ให้ "เดินหน้า" ได้ค่า + ทั้งสองข้าง)
static const bool INVERT_LEFT_ENC  = true;
static const bool INVERT_RIGHT_ENC = false;   // ถ้าเดินหน้าแล้วค่าขวาเป็นลบ ให้เปลี่ยนเป็น true

// ===== State =====
volatile long leftCount  = 0;
volatile long rightCount = 0;

static volatile bool lastLA = 0, lastLB = 0;
static volatile bool lastRA = 0, lastRB = 0;

static long lastLeftCount  = 0;
static long lastRightCount = 0;
static unsigned long lastSpeedCalc = 0;
static double leftCps  = 0.0;
static double rightCps = 0.0;

// ===== Fast add/sub with invert =====
static inline void addLeft(int d){
  leftCount += (INVERT_LEFT_ENC ? -d : d);
}
static inline void addRight(int d){
  rightCount += (INVERT_RIGHT_ENC ? -d : d);
}

/*
  Quadrature หลักการ: เมื่อ A หรือ B เปลี่ยน ให้ดูความสัมพันธ์ A==B เพื่อกำหนดทิศ
  - ถ้า A เปลี่ยน:  (A == B) ? ++ : --
  - ถ้า B เปลี่ยน:  (A != B) ? ++ : --
  หมายเหตุ: ขึ้นกับการเดินสาย/phase ของ encoder ถ้าทิศกลับ ให้สลับ INVERT_* ด้านบน
*/

// --- Left encoder ISRs ---
void IRAM_ATTR handleLeftA() {
  leftA_edges++;   
  bool a = digitalRead(ENC_L_A);
  bool b = lastLB; // ใช้ค่าบันทึกล่าสุดของ B (ลดเวลาอ่าน I/O อีกครั้ง)
  lastLA = a;
  if (a == b) addLeft(+1); else addLeft(-1);
}
void IRAM_ATTR handleLeftB() {
  
  bool b = digitalRead(ENC_L_B);
  bool a = lastLA;
  lastLB = b;
  if (a != b) addLeft(+1); else addLeft(-1);
}

// --- Right encoder ISRs ---
void IRAM_ATTR handleRightA() {
  rightA_edges++;
  bool a = digitalRead(ENC_R_A);
  bool b = lastRB;
  lastRA = a;
  if (a == b) addRight(+1); else addRight(-1);
}
void IRAM_ATTR handleRightB() {
  bool b = digitalRead(ENC_R_B);
  bool a = lastRA;
  lastRB = b;
  if (a != b) addRight(+1); else addRight(-1);
}

void encoderSetup() {
  // ถ้า encoder ของคุณมี open-collector/แบบไม่มี pull-up ภายนอก:
  // *ต้องมี* ตัวต้านทาน pull-up 10k ไป 3.3V สำหรับทั้ง A และ B ของซ้าย/ขวา
  // (เพราะขา 34–39 ไม่มี INPUT_PULLUP)
  pinMode(ENC_L_A, INPUT);
  pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT);
  pinMode(ENC_R_B, INPUT);

  // อ่านค่าเริ่มต้นกันกระตุกครั้งแรก
  lastLA = digitalRead(ENC_L_A);
  lastLB = digitalRead(ENC_L_B);
  lastRA = digitalRead(ENC_R_A);
  lastRB = digitalRead(ENC_R_B);

  // ใช้ interrupt ทั้ง A และ B จะนับละเอียดและทิศนิ่งกว่า
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), handleLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), handleLeftB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), handleRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), handleRightB, CHANGE);

  lastSpeedCalc = millis();
}

void encoderUpdate() {
  unsigned long now = millis();
  unsigned long dt = now - lastSpeedCalc;
  if (dt >= 100) { // อัปเดตทุก 100 ms
    long l = leftCount;
    long r = rightCount;
    leftCps  = (double)(l - lastLeftCount)  / (dt / 1000.0);
    rightCps = (double)(r - lastRightCount) / (dt / 1000.0);
    lastLeftCount  = l;
    lastRightCount = r;
    lastSpeedCalc  = now;
  }
}

long   getLeftCount()     { return leftCount; }
long   getRightCount()    { return rightCount; }
double getLeftSpeedCps()  { return leftCps; }
double getRightSpeedCps() { return rightCps; }
