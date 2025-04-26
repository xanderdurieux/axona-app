#include <ArduinoBLE.h>
#include <Arduino.h>

#include "BLEManager.hpp"
#include "CommandProcessor.hpp"
#include "IMUProcessor.hpp"

#define STATUS_CHECK 9
#define STATUS_LED 3

#define LEVEL_0_LED 10
#define LEVEL_1_LED 8
#define LEVEL_2_LED 6
#define LEVEL_3_LED 4
#define LEVEL_4_LED 2

BLEManager bleManager;
CommandProcessor& commandProcessor = CommandProcessor::getInstance(&bleManager);
IMUProcessor& imuProcessor = IMUProcessor::getInstance();

void setup() {
  Serial.begin(9600);
  while (!Serial);

  commandProcessor.printHelp();
  
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }
  Serial.println("BLE initialized.\n");

  pinMode(STATUS_CHECK, INPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(LEVEL_0_LED, OUTPUT);
  pinMode(LEVEL_1_LED, OUTPUT);
  pinMode(LEVEL_2_LED, OUTPUT);
  pinMode(LEVEL_3_LED, OUTPUT);
  pinMode(LEVEL_4_LED, OUTPUT);

  digitalWrite(STATUS_LED, LOW);  
  digitalWrite(LEVEL_0_LED, LOW);
  digitalWrite(LEVEL_1_LED, LOW);
  digitalWrite(LEVEL_2_LED, LOW);
  digitalWrite(LEVEL_3_LED, LOW);
  digitalWrite(LEVEL_4_LED, LOW);
}

void loop() {
  BLE.poll();  // Keep BLE stack active
  commandProcessor.processInput();

  // STATUS CHECK
  if (digitalRead(STATUS_CHECK) == HIGH) {
    digitalWrite(STATUS_LED, HIGH);
  } else {
    digitalWrite(STATUS_LED, LOW);
  }

  int impactLevel = imuProcessor.calculateImpactLevel();

  if (impactLevel > -1) {
    digitalWrite(LEVEL_0_LED, HIGH);
    digitalWrite(LEVEL_1_LED, impactLevel >= 1 ? HIGH : LOW);
    digitalWrite(LEVEL_2_LED, impactLevel >= 2 ? HIGH : LOW);
    digitalWrite(LEVEL_3_LED, impactLevel >= 3 ? HIGH : LOW);
    digitalWrite(LEVEL_4_LED, impactLevel >= 4 ? HIGH : LOW);   
  }
}
