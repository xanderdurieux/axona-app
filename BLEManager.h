#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <ArduinoBLE.h>
#include "IMUProcessor.h"

class BLEManager {
public:
  static BLEManager& getInstance() {
    static BLEManager instance;
    return instance;
  }

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
  BLEManager();
  BLEManager(const BLEManager&) = delete;
  BLEManager& operator=(const BLEManager&) = delete;

  static void notificationCallback(BLEDevice device, BLECharacteristic characteristic);

  IMUProcessor& imuProcessor;
  BLEDevice selectedDevice;
  BLECharacteristic selectedCharacteristic;
  BLEDevice scannedDevices[10];
  int deviceCount = 0;
};

#endif