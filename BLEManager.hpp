#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <ArduinoBLE.h>
#include <cstdint>
#include <cstring>

#include "IMUData.hpp"
#include "IMUProcessor.hpp"

#define MAX_DEVICES 10
#define MIN_RSSI -80
#define SCAN_TIME 5000

class DataView {
private:
    const uint8_t* buffer_;
    const size_t length_;

public:
    DataView(const uint8_t* buffer, size_t length)
        : buffer_(buffer), length_(length) {}

    bool checkBounds(size_t startIndex, size_t byteCount) const {
        return (startIndex + byteCount) <= length_;
    }

    uint8_t getUint8(size_t startIndex) const {
        if (!checkBounds(startIndex, 1)) return 0;
        return buffer_[startIndex];
    }

    uint16_t getUint16(size_t startIndex) const {
        if (!checkBounds(startIndex, 2)) return 0;
        uint16_t value;
        memcpy(&value, buffer_ + startIndex, sizeof(value));
        return value;
    }

    uint32_t getUint32(size_t startIndex) const {
        if (!checkBounds(startIndex, 4)) return 0;
        uint32_t value;
        memcpy(&value, buffer_ + startIndex, sizeof(value));
        return value;
    }

    int32_t getInt32(size_t startIndex) const {
        if (!checkBounds(startIndex, 4)) return 0;
        int32_t value;
        memcpy(&value, buffer_ + startIndex, sizeof(value));
        return value;
    }

    float getFloat32(size_t startIndex) const {
        if (!checkBounds(startIndex, 4)) return 0.0f;
        float value;
        memcpy(&value, buffer_ + startIndex, sizeof(value));
        return value;
    }
};

class BLEManager {
public:
  BLEManager(): deviceCount(0) {};
  
  bool begin();
  void poll();
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

  bool isSubscribed = false;
  BLEDevice selectedDevice;
  BLECharacteristic selectedCharacteristic;
  BLEDevice scannedDevices[10];
  int deviceCount;
};

#endif