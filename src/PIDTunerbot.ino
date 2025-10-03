// SBR.ino
#include <Arduino.h>
#include "motor.h"
#include "espnow.h"
//#include "display.h"
#include <Wire.h>

#include <MPU6050_light.h>
#include "encoder.h"
#include "webui.h"
#include <Preferences.h>

MPU6050 mpu(Wire);

// I2C pins for ESP32 (change if your wiring uses different pins)
const int I2C_SDA = 21; // SDA pin
const int I2C_SCL = 22; // SCL pin

// PID control variables
double setpoint = 3;  // Desired angle (upright position)
double input = 0.0;   // Current angle from IMU
double output = 0.0;  // PID controller output

// PID tuning parameters (to be adjusted via ESP-NOW)
double Kp = 20.0;
double Ki = 0.0;
double Kd = 0.3;


// Speed PID (inner loop) parameters (per motor)
double Kp_speed_L = 1.0;
double Ki_speed_L = 0.0;
double Kd_speed_L = 0.0;

double Kp_speed_R = 1.0;
double Ki_speed_R = 0.0;
double Kd_speed_R = 0.0;

// Speed PID state (per motor)
double speedPrevErrorL = 0.0;
double speedIntegralL = 0.0;
double speedPrevErrorR = 0.0;
double speedIntegralR = 0.0;

// Target speed from angle PID (counts per second)
double targetSpeedCps = 0.0;

// User motion commands (from web UI)
double userTargetCps = 0.0; // forward/back speed command (counts/sec)
double userTurnCps = 0.0;   // turn command (positive = turn right)

// User command limits
double MaxUserSpeed = 100.0; // counts/sec for forward/back
double MaxUserTurn = 80.0;   // counts/sec differential for turning

// Last motor commands (for web UI/status)
double lastMotorCmdL = 0.0;
double lastMotorCmdR = 0.0;

// PID calculation variables
double previousError = 0.0;
double integral = 0.0;
unsigned long lastComputeTime = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    delay(10);  // Wait for Serial to initialize
  }


  // Initialize the MPU6050 sensor (explicit SDA/SCL pins)
  Wire.begin(I2C_SDA, I2C_SCL);
  mpu.begin();
  delay(100);
  // Optional: calibrate gyroscope (prints progress)
  mpu.calcGyroOffsets();
  Serial.println("MPU6050_light initialized successfully!");

  // Initialize motor
  motorSetup();

  // Initialize encoders
  encoderSetup();

  // Load saved PID defaults
  loadDefaults();

  // Initialize last compute time
  lastComputeTime = millis();

  // Initialize ESP-NOW
  espnowSetup();

  // Initialize Web UI
  webuiSetup();

  // Initialize Display (if used)
  // displaySetup();
}

void loop() {
  // Read sensor data every 10ms
  static unsigned long lastSensorReadTime = 0;
  if (millis() - lastSensorReadTime >= 10) {
    lastSensorReadTime = millis();


  // Update MPU6050_light and get fused pitch angle (degrees)
  mpu.update();
  input = mpu.getAngleX();

    // Update encoders (compute speeds)
    encoderUpdate();

  // Read measured speeds (left and right)
  double measuredL = getLeftSpeedCps();
  double measuredR = getRightSpeedCps();

  // Angle PID computes a virtual output -> convert to targetSpeed (simple mapping)
  const double MaxPIDOutput = 255.0;
  const double MaxTargetSpeed = 200.0; // counts per second, tune for your robot
  targetSpeedCps = (output / MaxPIDOutput) * MaxTargetSpeed;

  // Combine angle-derived target with user motion commands
  // userTargetCps is added equally to both wheels for forward/back
  // userTurnCps is added differentially: right wheel += userTurnCps/2, left -= userTurnCps/2
  double targetL = targetSpeedCps + userTargetCps - userTurnCps * 0.5;
  double targetR = targetSpeedCps + userTargetCps + userTurnCps * 0.5;

  // Compute per-motor speed PID
  double currentTime = millis();
  static double lastSpeedPidTime = 0;
  double dt_speed = (currentTime - lastSpeedPidTime) / 1000.0;
  if (dt_speed <= 0) dt_speed = 0.001;
  lastSpeedPidTime = currentTime;

  double errorL = targetL - measuredL;
  speedIntegralL += errorL * dt_speed;
  double derivL = (errorL - speedPrevErrorL) / dt_speed;
  double cmdL = Kp_speed_L * errorL + Ki_speed_L * speedIntegralL + Kd_speed_L * derivL;
  speedPrevErrorL = errorL;

  double errorR = targetR - measuredR;
  speedIntegralR += errorR * dt_speed;
  double derivR = (errorR - speedPrevErrorR) / dt_speed;
  double cmdR = Kp_speed_R * errorR + Ki_speed_R * speedIntegralR + Kd_speed_R * derivR;
  speedPrevErrorR = errorR;

  // Constrain commands
  cmdL = constrain(cmdL, -255, 255);
  cmdR = constrain(cmdR, -255, 255);

  // Apply motor commands per motor (store for UI)
  lastMotorCmdL = cmdL;
  lastMotorCmdR = cmdR;
  updateMotorControl(cmdL, cmdR);

    // Compute outer PID (for telemetry and tuning)
    computePID();

    // Send PID data via ESP-NOW
    sendPIDData();

    // Debugging output (optional)
    Serial.print("Setpoint:");
    Serial.print(setpoint);
    Serial.print(",");
    Serial.print("Pitch:");
    Serial.print(input);
    Serial.print(",");
    Serial.print("PID Output:");
    Serial.println(output);
  }

  // Web UI loop
  webuiLoop();

  // For DC motor (MDD10A) no step pulses are needed
  // generateStepPulses();
}

void computePID() {
  unsigned long currentTime = millis();
  double dt = (currentTime - lastComputeTime) / 1000.0;  // Convert to seconds
  lastComputeTime = currentTime;

  // Avoid division by zero
  if (dt <= 0)
    dt = 0.001;

  // Calculate error
  double error = setpoint - input;

  // Calculate integral
  integral += error * dt;
  // Apply integral windup protection
  if (integral > 3) {
    integral = 3;
  }
  if (integral < -3) {
    integral = -3;
  }

  // Calculate derivative
  double derivative = (error - previousError) / dt;

  // Compute PID output
  output = Kp * error + Ki * integral + Kd * derivative;

  // Save error for next loop
  previousError = error;

  // Constrain output
  if (output > 255) {
    output = 255;
  }
  if (output < -255) {
    output = -255;
  }

  // Populate pidData for sending
  pidData.p = Kp * error;
  pidData.i = Ki * integral;
  pidData.d = Kd * derivative;
  pidData.setpoint = setpoint;
  pidData.input = input;
  pidData.output = output;

  // Debugging output (optional)
  Serial.print("P:");
  Serial.print(pidData.p);
  Serial.print(",");
  Serial.print("I:");
  Serial.print(pidData.i);
  Serial.print(",");
  Serial.print("D:");
  Serial.print(pidData.d);
  Serial.print(",");
}

// Preferences for saving/loading defaults
void loadDefaults() {
  Preferences prefs;
  prefs.begin("robot", true); // read-only
  Kp = prefs.getDouble("Kp", Kp);
  Ki = prefs.getDouble("Ki", Ki);
  Kd = prefs.getDouble("Kd", Kd);
  Kp_speed_L = prefs.getDouble("Kp_sL", Kp_speed_L);
  Ki_speed_L = prefs.getDouble("Ki_sL", Ki_speed_L);
  Kd_speed_L = prefs.getDouble("Kd_sL", Kd_speed_L);
  Kp_speed_R = prefs.getDouble("Kp_sR", Kp_speed_R);
  Ki_speed_R = prefs.getDouble("Ki_sR", Ki_speed_R);
  Kd_speed_R = prefs.getDouble("Kd_sR", Kd_speed_R);
  prefs.end();
}

void saveDefaults() {
  Preferences prefs;
  prefs.begin("robot", false);
  prefs.putDouble("Kp", Kp);
  prefs.putDouble("Ki", Ki);
  prefs.putDouble("Kd", Kd);
  prefs.putDouble("Kp_sL", Kp_speed_L);
  prefs.putDouble("Ki_sL", Ki_speed_L);
  prefs.putDouble("Kd_sL", Kd_speed_L);
  prefs.putDouble("Kp_sR", Kp_speed_R);
  prefs.putDouble("Ki_sR", Ki_speed_R);
  prefs.putDouble("Kd_sR", Kd_speed_R);
  prefs.end();
}

