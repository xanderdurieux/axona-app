#include <Arduino.h>

#include "BLEManager.hpp"

#include "CommandProcessor.hpp"
#include "IMUProcessor.hpp"

#define LEVEL_0_LED 11
#define LEVEL_1_LED 10
#define LEVEL_2_LED 9
#define LEVEL_3_LED 8
#define LEVEL_4_LED 7

#define NFC_WRITE_LED 6 

BLEManager bleManager;
CommandProcessor& commandProcessor = CommandProcessor::getInstance(&bleManager);
IMUProcessor& imuProcessor = IMUProcessor::getInstance();

void setup() {
  Serial.begin(9600);
  while (!Serial);

  commandProcessor.printHelp();
  
  if (!bleManager.begin()) {
    while(1);
  }

  // Setup LEDs
  pinMode(LEVEL_0_LED, OUTPUT);
  pinMode(LEVEL_1_LED, OUTPUT);
  pinMode(LEVEL_2_LED, OUTPUT);
  pinMode(LEVEL_3_LED, OUTPUT);
  pinMode(LEVEL_4_LED, OUTPUT);
  pinMode(NFC_WRITE_LED, OUTPUT);

  digitalWrite(LEVEL_0_LED, LOW);
  digitalWrite(LEVEL_1_LED, LOW);
  digitalWrite(LEVEL_2_LED, LOW);
  digitalWrite(LEVEL_3_LED, LOW);
  digitalWrite(LEVEL_4_LED, LOW);
  digitalWrite(NFC_WRITE_LED, LOW);
}

void loop() {
  bleManager.poll();

  commandProcessor.processInput();

  int impactLevel = imuProcessor.calculateImpactLevel();

  if (impactLevel > -1) {
    digitalWrite(LEVEL_0_LED, HIGH);
    digitalWrite(LEVEL_1_LED, impactLevel >= 1 ? HIGH : LOW);
    digitalWrite(LEVEL_2_LED, impactLevel >= 2 ? HIGH : LOW);
    digitalWrite(LEVEL_3_LED, impactLevel >= 3 ? HIGH : LOW);
    digitalWrite(LEVEL_4_LED, impactLevel >= 4 ? HIGH : LOW);   
  }
}
