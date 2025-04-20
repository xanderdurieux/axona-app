#include <ArduinoBLE.h>
#include <Arduino.h>

#include "BLEManager.hpp"
#include "CommandProcessor.hpp"
#include "IMUProcessor.hpp"

#define STATUS_CHECK 9
#define STATUS_LED 3

#define IMU_LEVEL_0 10
#define IMU_LEVEL_1 8
#define IMU_LEVEL_2 6
#define IMU_LEVEL_3 4
#define IMU_LEVEL_4 2

BLEManager bleManager;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  CommandProcessor& commandProcessor = CommandProcessor::getInstance(&bleManager);
  commandProcessor.printHelp();
  
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }
  Serial.println("BLE initialized.\n");

  pinMode(STATUS_CHECK, INPUT);
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);  
}

void loop() {
  BLE.poll();  // Keep BLE stack active
  CommandProcessor::getInstance().processInput();

  // STATUS CHECK
  if (digitalRead(STATUS_CHECK) == HIGH) {
    digitalWrite(STATUS_LED, HIGH);
  } else {
    digitalWrite(STATUS_LED, LOW);
  }
}
