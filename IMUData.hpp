#ifndef IMU_DATA_H
#define IMU_DATA_H

#include <Arduino.h>
#include <cstdint>
#include <cstring>

struct IMUData {
  uint32_t timestamp;
  
  // Accelerometer (m/s²)
  float accX;
  float accY;
  float accZ;
  
  // Gyroscope (rad/s)
  float gyroX;
  float gyroY;
  float gyroZ;

  String toString() const {
    String result = "AccX:" + String(accX) + " AccY:" + String(accY) + " AccZ:" + String(accZ) +
                    " GyroX:" + String(gyroX) + " GyroY:" + String(gyroY) + " GyroZ:" + String(gyroZ);
    return result;
  }

  float getAcceleration() const {
    return sqrt(accX * accX + accY * accY + accZ * accZ);
  }
};

#endif
