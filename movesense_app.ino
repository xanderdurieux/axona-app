#include <Arduino.h>

#include "BLEManager.hpp"
#include "CommandProcessor.hpp"
#include "IMUProcessor.hpp"
// #include "RFIDManager.hpp"

#define STATUS_READ 12
#define STATUS_LED 13

#define LEVEL_0_LED 11
#define LEVEL_1_LED 10
#define LEVEL_2_LED 9
#define LEVEL_3_LED 8
#define LEVEL_4_LED 7 

#define RFID_SS_PIN 6
#define RFID_SCK_PIN 5
#define RFID_MOSI_PIN 4
#define RFID_MISO_PIN 3
#define RFID_RST_PIN 2

int counter = 0;

BLEManager bleManager;
// RFIDManager rfidManager(RFID_SS_PIN, RFID_RST_PIN); 

CommandProcessor& commandProcessor = CommandProcessor::getInstance(&bleManager);
IMUProcessor& imuProcessor = IMUProcessor::getInstance();

void setup() {
  Serial.begin(9600);
  while (!Serial);

  commandProcessor.printHelp();
  
  if (!bleManager.begin()) {
    while (1);
  }

  // Setup LEDs
  pinMode(STATUS_READ, INPUT);
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
  if (digitalRead(STATUS_READ) == HIGH) {
    digitalWrite(STATUS_LED, HIGH);
  } else {
    digitalWrite(STATUS_LED, LOW);
  }

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

  if (impactLevel > -1 ) {
    if (counter++ > 50) {
      counter = 0;
      // Debugging metrics to Serial Plotter
      Serial.print("ImpactLevel:");
      Serial.print(impactLevel);
      Serial.print(" PeakLinearAcc:");
      Serial.print(imuProcessor.getPeakLinearAcc());
      Serial.print(" GaddSI:");
      Serial.print(imuProcessor.getGaddSI());
      Serial.print(" HIC:");
      Serial.print(imuProcessor.getHIC(15)); // Example window of 15ms
      Serial.print(" PeakAngularAcc:");
      Serial.print(imuProcessor.getPeakAngularAcc());
      Serial.print(" BrIC:");
      Serial.print(imuProcessor.getBrIC(1.0, 1.0, 1.0)); // Example omegaC values
      Serial.print(" RIC:");
      Serial.println(imuProcessor.getRIC(15)); // Example window of 15ms
    }
  }
}