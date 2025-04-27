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

  // --- New metric computations ---
  // Peak linear acceleration (m/s²)
  double getPeakLinearAcc() const;
  // Gadd Severity Index (SI)
  double getGaddSI() const;
  // Head Injury Criterion over a window (ms)
  double getHIC(double windowMs = 15.0) const;
  // Peak angular acceleration (rad/s²)
  double getPeakAngularAcc() const;
  // Brain Injury Criterion (BrIC) with critical omegas
  double getBrIC(double omegaCX, double omegaCY, double omegaCZ) const;
  // Rotational Injury Criterion over a window (ms)
  double getRIC(double windowMs = 36.0) const;

private:
  IMUProcessor();
  IMUProcessor(const IMUProcessor&) = delete;
  IMUProcessor& operator=(const IMUProcessor&) = delete;

  std::deque<IMUData> imuDataBuffer;
  // Helper: vector of angular accel samples (time, magnitude)
  struct AngularSample { double time; double alpha; };
};

#endif