#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <ArduinoBLE.h>
#include "IMUProcessor.hpp"

#define SCAN_TIME 5000 // Scan duration in milliseconds
#define MAX_DEVICES 100 // Maximum number of devices to scan
#define MIN_RSSI -70  // Minimum RSSI value to consider a device

class BLEManager {
public:
  BLEManager(): deviceCount(0) {};

  void scanDevices();
  void listDevices();
  bool selectDevice(int index);
  void listServicesAndCharacteristics();
  bool subscribeCharacteristic(int sIndex, int cIndex);
  bool unsubscribeCharacteristic(int sIndex, int cIndex);
  bool readCharacteristic(int sIndex, int cIndex);
  bool writeCharacteristic(int sIndex, int cIndex, const uint8_t *data, int length);
  bool sendMovesenseCommand(int sIndex);
  void disconnect();

private:
  bool deviceAlreadyListed(BLEDevice device);

  static void notificationCallback(BLEDevice device, BLECharacteristic characteristic);

  BLEDevice selectedDevice;
  BLECharacteristic selectedCharacteristic;
  BLEDevice scannedDevices[10];
  int deviceCount;
};

#endif