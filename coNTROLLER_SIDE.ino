#include <esp_now.h>
#include <WiFi.h>

// Structure to hold the incoming message
typedef struct struct_message {
  char message[250];
} struct_message;

// Create a struct_message instance to receive data
struct_message incomingMessage;

// Callback when data is received
void onDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy(&incomingMessage, incomingData, sizeof(incomingMessage));

  // Print the received message to the Serial Monitor
  Serial.print("Message received: ");
  Serial.println(incomingMessage.message);
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callback for receiving data
  esp_now_register_recv_cb(onDataRecv);
}

void loop() {
  // Do nothing, waiting for messages
}
