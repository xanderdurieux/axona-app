#include <ArduinoBLE.h>
#include "BLEManager.h"
#include "CommandProcessor.h"

BLEManager bleManager;
CommandProcessor commandProcessor(bleManager);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  commandProcessor.printHelp();
  
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }
  Serial.println("BLE initialized.\n");
}

void loop() {
  BLE.poll();  // Keep BLE stack active
  commandProcessor.processInput();
}
