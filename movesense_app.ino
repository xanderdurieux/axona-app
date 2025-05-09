#include <Arduino.h>
#include <ArduinoBLE.h>

#include "BLEManager.hpp"
#include "IMUProcessor.hpp"

#define DEBUG

#define LED_PIN_1 11
#define LED_PIN_2 9
#define LED_PIN_3 7
#define LED_PIN_4 5
#define LED_PIN_5 3

BLEManager bleManager;
IMUProcessor& imuProcessor = IMUProcessor::getInstance();

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);

  // Initialize LED pins
  for (int i = 2; i <= 6; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  if (!bleManager.begin()) {
    Serial.println("Failed to initialize BLE");
    while (1);
  }
  Serial.println("BLE initialized successfully");

  bleManager.scanDevices();
  bleManager.listDevices();

  const char* targetAddress = "74:92:ba:10:e8:23";
  int targetIndex = bleManager.getDeviceIndex(targetAddress);
  if (targetIndex == -1) {
    Serial.println("Target device not found.");
    return;
  }

  if (!bleManager.selectDevice(targetIndex)) {
    Serial.println("Failed to select device.");
    return;
  }
  Serial.println("Device selected successfully");
  
  String sampleRate = "52";
  const uint8_t subscribeCommand[] = {1, 99, '/', 'M', 'e', 'a', 's', '/', 'I', 'M', 'U', '6', '/', sampleRate.charAt(0), sampleRate.charAt(1), sampleRate.charAt(2), sampleRate.charAt(3)};
  int commandLength = 13 + sampleRate.length();

  if(!bleManager.writeCharacteristic(5, 0, subscribeCommand, commandLength)) {
    Serial.println("Failed to write characteristic");
    return;
  }
  if(!bleManager.subscribeCharacteristic(5, 1)) {
    Serial.println("Failed to subscribe to characteristic");
    return;
  }
  Serial.println("Subscribed to IMU sensor");  
}

void loop() {
  bleManager.poll();

  // Get impact level (0-4)
  if (bleManager.isSubscribed()) {
    int impactLevel = imuProcessor.getImpactLevel();
  
    // Update LEDs based on impact level
    digitalWrite(LED_PIN_1, impactLevel >= 0 ? HIGH : LOW);
    digitalWrite(LED_PIN_2, impactLevel >= 1 ? HIGH : LOW);
    digitalWrite(LED_PIN_3, impactLevel >= 2 ? HIGH : LOW);
    digitalWrite(LED_PIN_4, impactLevel >= 3 ? HIGH : LOW);
    digitalWrite(LED_PIN_5, impactLevel >= 4 ? HIGH : LOW);

    // If impact detected (level > 0), print metrics
    if (impactLevel > 0) {
      Serial.println("\n--- Impact Detected ---");
      Serial.print("Impact Level: ");
      Serial.println(impactLevel);
      
      // Get and print HIC (Head Injury Criterion)
      double hic = imuProcessor.getHIC(15.0);  // 15ms window
      Serial.print("HIC: ");
      Serial.println(hic);
      
      // Get and print BrIC (Brain Injury Criterion)
      // Using standard critical angular velocities (rad/s)
      double bric = imuProcessor.getBrIC(66.3, 66.3, 66.3);
      Serial.print("BrIC: ");
      Serial.println(bric);
      
      // Determine concussion risk based on HIC and BrIC
      String concussionRisk = "Low";
      if (hic > 1000 || bric > 0.85) {
        concussionRisk = "High";
      } else if (hic > 500 || bric > 0.65) {
        concussionRisk = "Medium";
      }
      Serial.print("Concussion Risk: ");
      Serial.println(concussionRisk);
      
      // Get and print peak linear acceleration
      double peakAcc = imuProcessor.getPeakLinearAcc();
      Serial.print("Peak Acceleration: ");
      Serial.print(peakAcc);
      Serial.println(" m/s²");
      
      // Get and print velocities
      double velBefore = imuProcessor.getVelocityBeforeImpact();
      double velAfter = imuProcessor.getVelocityAfterImpact();
      Serial.print("Velocity Before Impact: ");
      Serial.print(velBefore);
      Serial.println(" m/s");
      Serial.print("Velocity After Impact: ");
      Serial.print(velAfter);
      Serial.println(" m/s");
      
      Serial.println("----------------------\n");
    }
  }

  delay(1000);
}
