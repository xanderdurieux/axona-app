#ifndef IMUPROCESSOR_H
#define IMUPROCESSOR_H

#include <Arduino.h>
#include <math.h>
#include "IMUData.h"

#define IMPACT_THRESHOLD 5.0

class IMUProcessor {
public:
  static IMUProcessor& getInstance() {
    static IMUProcessor instance;
    return instance;
  }

  void processIMUData(const IMUData& imu);

private:
  IMUProcessor();
  IMUProcessor(const IMUProcessor&) = delete;
  IMUProcessor& operator=(const IMUProcessor&) = delete;

  float buffer[10][9]; // Buffer for downsampling (10 samples, 9 values per sample)
  int bufferIndex;
  float calculateImpact(const IMUData& imu);
  void downsampleAndStore(const IMUData& imu);
  void resetBuffer();
};

#endif
