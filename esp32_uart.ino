#include <Arduino.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

int32_t leftSpeed = 0;
int32_t rightSpeed = 0;
uint8_t ir_val = 0;
float yaw = 0;

void setup() {
  Serial.begin(115200); // For debug
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // STM32 -> ESP32 UART (adjust RX/TX pins)

  if (!SerialBT.begin("ESP32_UART_Bridge")) {
    Serial.println("Bluetooth initialization failed");
    while (1);
  }
  Serial.println("Bluetooth started. Pair with 'ESP32_UART_Bridge'");
}

void loop() {
  const size_t dataSize = sizeof(int32_t)*2 + sizeof(uint8_t) + sizeof(float);
  static uint8_t buffer[dataSize];
  static size_t received = 0;

  while (Serial2.available() && received < dataSize) {
    buffer[received++] = Serial2.read();
  }

  if (received == dataSize) {
    memcpy(&leftSpeed, buffer, sizeof(leftSpeed));
    memcpy(&rightSpeed, buffer + sizeof(leftSpeed), sizeof(rightSpeed));
    memcpy(&ir_val, buffer + sizeof(leftSpeed) + sizeof(rightSpeed), sizeof(ir_val));
    memcpy(&yaw, buffer + sizeof(leftSpeed) + sizeof(rightSpeed) + sizeof(ir_val), sizeof(yaw));

    // Format as text (optional: use JSON or CSV as needed)
    String output = String("Left: ") + leftSpeed +
                    ", Right: " + rightSpeed +
                    ", IR: " + ir_val +
                    ", Yaw: " + yaw + "\n";

    SerialBT.print(output); // Send to PC via Bluetooth SPP
    Serial.print(output);   // Debug to Serial Monitor

    received = 0;
  }
}
