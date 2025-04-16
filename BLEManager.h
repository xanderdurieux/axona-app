#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <Arduino.h>
#include <ArduinoBLE.h>

#define MAX_DEVICES 200
#define SCAN_TIME 7000

class BLEManager {
public:
  BLEManager();
  
  void scanDevices();
  void listDevices();
  bool selectDevice(int index);
  void listServicesAndCharacteristics();
  bool subscribeCharacteristic(int sIndex, int cIndex);
  bool readCharacteristic(int sIndex, int cIndex);
  bool writeCharacteristic(int sIndex, int cIndex, const uint8_t data, int length);
  bool sendMovesenseCommand(int sIndex);
  void disconnect();

private:
  BLEDevice scannedDevices[MAX_DEVICES];
  int deviceCount;
  BLEDevice selectedDevice;
  BLECharacteristic selectedCharacteristic;

  // Private helper methods
  bool deviceAlreadyListed(BLEDevice device);
  uint8_t hexCharToByte(char c);
  int parseHexString(const String &hexStr, uint8_t *buffer, int bufferSize);
  
  // Static callback for notifications
  static void notificationCallback(BLEDevice device, BLECharacteristic characteristic);
  };

#endif
