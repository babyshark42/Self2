// espnow.cpp
#include "espnow.h"

// External variables from the main file
extern double Kp;
extern double Ki;
extern double Kd;
extern double setpoint;

// Create a struct_message called myData (for receiving data)
struct_message myData;

// Create pidData (for sending data)
pid_data_t pidData;

// Transmitter Controller's MAC Address (replace with actual MAC address)
uint8_t controllerAddress[] = {0x10, 0x06, 0x1c, 0x97, 0xb3, 0xd8}; // Replace with your transmitter's MAC

// Callback function that will be executed when data is received
// void OnDataRecv(const esp_now_recv_info_t* esp_now_info, const uint8_t* incomingData, int len) {
//   memcpy(&myData, incomingData, sizeof(myData));

//   // Update PID parameters
//   Kp = myData.Kp;
//   Ki = myData.Ki;
//   setpoint = myData.Kd / 100 * 5 + 1;

//   // Print the received values
//   Serial.print("Received new PID values - Kp: ");
//   Serial.print(Kp);
//   Serial.print(" Ki: ");
//   Serial.print(Ki);
//   Serial.print(" Setpoint: ");
//   Serial.println(setpoint);
// }

// Function to send PID data to the transmitter controller
void sendPIDData() {
  esp_err_t result = esp_now_send(controllerAddress, (uint8_t*)&pidData, sizeof(pidData));

  if (result == ESP_OK) {
    // Data sent successfully
  } else {
    Serial.print("Error sending PID data: ");
    Serial.println(result);
  }
}

void espnowSetup() {
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register for receive callback to get packet info
  //esp_now_register_recv_cb(OnDataRecv);

  // Register for send callback (optional)
  // esp_now_register_send_cb(OnDataSent);

  Serial.println("ESP-NOW initialized and ready");

  // Register peer for sending data
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, controllerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}
