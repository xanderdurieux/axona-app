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
  Serial.begin(115200);
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

    static unsigned long lastImpactTime = 0;
    static int lastImpactLevel = 0;
    const unsigned long LED_DURATION = 3000; // LEDs stay on for 1 second
    
    if (impactLevel > 0) {
        lastImpactTime = millis();
        lastImpactLevel = impactLevel;
    }
    
    // Update LEDs based on impact level and timing
    if (millis() - lastImpactTime < LED_DURATION) {
        digitalWrite(LED_PIN_1, lastImpactLevel >= 0 ? HIGH : LOW);
        digitalWrite(LED_PIN_2, lastImpactLevel >= 1 ? HIGH : LOW);
        digitalWrite(LED_PIN_3, lastImpactLevel >= 2 ? HIGH : LOW);
        digitalWrite(LED_PIN_4, lastImpactLevel >= 3 ? HIGH : LOW);
        digitalWrite(LED_PIN_5, lastImpactLevel >= 4 ? HIGH : LOW);
    } else {
        // Turn off all LEDs after duration
        digitalWrite(LED_PIN_2, LOW);
        digitalWrite(LED_PIN_3, LOW);
        digitalWrite(LED_PIN_4, LOW);
        digitalWrite(LED_PIN_5, LOW);
    }
      
    if (impactLevel > 0) {
      Serial.println("--- Impact Detected ---");
      Serial.print("Impact Level: ");
      Serial.println(impactLevel);

      // Get and print HIC (Head Injury Criterion)
      double hic = imuProcessor.getHIC(15.0);  // 15ms window
      Serial.println("hicData||" + String(hic, 2));     

      // Determine concussion risk based on HIC
      String concussionRisk = "Low";
      if (hic > 1000) {
        concussionRisk = "High";
      } else if (hic > 500) {
        concussionRisk = "Medium";
      }
      Serial.println("concussionRisk||" + concussionRisk);

      // Get peak linear acceleration
      double peakAcc = imuProcessor.getPeakLinearAcc();
      Serial.println("peakAcc||" + String(peakAcc, 2) + " g");

      // Get riding velocity before impact
      double ridingVelocity = imuProcessor.getRidingVelocitybeforeImpact();
      Serial.println("RidingVelocity||" + String(ridingVelocity, 2) + " km/h");      

      // Get head velocity on impact
      double headVelocity = imuProcessor.getHeadVelocityOnImpact();
      Serial.println("HeadVelocity||" + String(headVelocity, 2) + " km/h");

      Serial.println("----------------------\n");
    }
  }
}
