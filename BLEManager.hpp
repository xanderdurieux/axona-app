#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <ArduinoBLE.h>
#include "IMUData.hpp"

#define MAX_DEVICES 10
#define MIN_RSSI -80
#define SCAN_TIME 5000

#define DATA 0x01
#define DATA_PART2 0x02
#define DATA_PART3 0x03

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
  void disconnect();
    
  int getDeviceIndex(String address);

private:
  bool deviceAlreadyListed(BLEDevice device);

  static void notificationCallback(BLEDevice device, BLECharacteristic characteristic);

  BLEDevice selectedDevice;
  BLECharacteristic selectedCharacteristic;
  BLEDevice scannedDevices[10];
  int deviceCount;

  bool isSubscribed = false;
};

#endif