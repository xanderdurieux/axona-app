#ifndef IMU_PROCESSOR_H
#define IMU_PROCESSOR_H

#include <vector>
#include <deque>
#include <cmath>
#include <iostream>
#include <limits>

#include "IMUData.hpp"

#define IMPACT_THRESHOLD_LOW 2.5
#define IMPACT_THRESHOLD_MEDIUM 5.0
#define IMPACT_THRESHOLD_HIGH 7.5
#define IMPACT_THRESHOLD_SEVERE 10.0

class IMUProcessor {
public:
  static IMUProcessor& getInstance() {
    static IMUProcessor instance;
    return instance;
  }

  void processData(
    float accX, float accY, float accZ, 
    float gyroX, float gyroY, float gyroZ,
    uint32_t timestamp
  );
  void clearData();
  
  int calculateImpactLevel() const;

private:
  IMUProcessor();
  IMUProcessor(const IMUProcessor&) = delete;
  IMUProcessor& operator=(const IMUProcessor&) = delete;

  std::deque<IMUData> imuDataBuffer;
};

#endif