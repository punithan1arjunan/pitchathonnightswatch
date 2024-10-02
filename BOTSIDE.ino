#include <esp_now.h>
#include <WiFi.h>
#include <BluetoothSerial.h>  // Bluetooth library for ESP32

BluetoothSerial SerialBT;     // Create a Bluetooth Serial object

// Structure to hold the message
typedef struct struct_message {
  char message[250];  // Size of the message buffer
} struct_message;

// Create a struct_message instance to send
struct_message outgoingMessage;
struct_message incomingMessage;  // Create a struct for receiving messages

// MAC Address of the peer ESP32 (You need to swap these on both ESPs)
uint8_t peerAddress[] = {0xc8, 0x2e, 0x18, 0x67, 0xa3, 0x58}; // Use the MAC address of the other ESP32

// Callback when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Updated Callback when data is received
void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  // Copy the incoming data into the incomingMessage struct
  memcpy(&incomingMessage, incomingData, sizeof(incomingMessage));
  
  // Print the received message to Serial Monitor
  Serial.print("Received message from: ");
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X", recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2], recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
  Serial.print(macStr);
  Serial.print(" - Message: ");
  Serial.println(incomingMessage.message);

  // Send the received message via Bluetooth Serial
  SerialBT.println(incomingMessage.message);  // Send received message over Bluetooth
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Initialize Bluetooth Serial
  if (!SerialBT.begin("ESP32_BT")) {  // Set Bluetooth device name
    Serial.println("An error occurred initializing Bluetooth");
  } else {
    Serial.println("Bluetooth initialized. You can pair with ESP32_BT");
  }

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the send callback function
  esp_now_register_send_cb(onDataSent);

  // Register the receive callback function
  esp_now_register_recv_cb(onDataRecv);

  // Add the peer to the system
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("ESP32 ready for dual-way communication. Enter a message to send:");
}

void loop() {
  // Check if data is available on Serial Monitor
  if (Serial.available() > 0) {
    // Read the input message from Serial Monitor
    String input = Serial.readStringUntil('\n');
    input.trim();

    // Check if input is valid and not empty
    if (input.length() > 0) {
      // Copy the input message into the outgoingMessage struct
      input.toCharArray(outgoingMessage.message, sizeof(outgoingMessage.message));

      // Send message via ESP-NOW to the peer
      esp_err_t result = esp_now_send(peerAddress, (uint8_t *)&outgoingMessage, sizeof(outgoingMessage));

      if (result == ESP_OK) {
        Serial.println("Message sent successfully");
      } else {
        Serial.println("Error sending the message");
      }

      // Send the message via Bluetooth Serial as well
      SerialBT.println(input);  // Send the message over Bluetooth
      Serial.println("Message sent via Bluetooth");
    }
  }

  // Add some delay to avoid overwhelming the loop
  delay(100);
}
