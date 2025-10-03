// espnow.h
#ifndef ESPNOW_H
#define ESPNOW_H

#include <esp_now.h>
#include <WiFi.h>

// Structure to receive data (from transmitter)
typedef struct struct_message {
  float Ki;
  float Kp;
  float Kd;
} struct_message;

// Structure to send data (to transmitter)
typedef struct pid_data_t {
  float p;
  float i;
  float d;
  float setpoint;
  float input;
  float output;
} pid_data_t;

// External variables
extern struct_message myData;
extern pid_data_t pidData;

// Function prototypes
void espnowSetup();
//void OnDataRecv(const esp_now_recv_info_t* esp_now_info, const uint8_t* incomingData, int len);
void sendPIDData();

#endif  // ESPNOW_H
