#include <ArduinoBLE.h>
#include <Arduino.h>

#include "BLEManager.h"
#include "CommandProcessor.h"
#include "IMUProcessor.h"

BLEManager bleManager;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  CommandProcessor& commandProcessor = CommandProcessor::getInstance(&bleManager);
  commandProcessor.printHelp();
  
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }
  Serial.println("BLE initialized.\n");
}

void loop() {
  BLE.poll();  // Keep BLE stack active
  CommandProcessor::getInstance().processInput();
}
