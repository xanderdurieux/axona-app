#ifndef IMUDATA_H
#define IMUDATA_H

#include <Arduino.h>

// Struct to represent one sample of IMU data.
struct IMUData {
  uint32_t timestamp; // Timestamp (e.g., in milliseconds or seconds)
  float accX, accY, accZ;
  float gyroX, gyroY, gyroZ;
  float magX, magY, magZ;

  String toString() const {
    return String(timestamp) + " - ACC: x=" + String(accX, 2) + " y=" + String(accY, 2) + " z=" + String(accZ, 2) +
         " - GYRO: x=" + String(gyroX, 2) + " y=" + String(gyroY, 2) + " z=" + String(gyroZ, 2) +
         " - MAG: x=" + String(magX, 2) + " y=" + String(magY, 2) + " z=" + String(magZ, 2);
  }
};

#endif
