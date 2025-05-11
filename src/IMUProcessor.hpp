#ifndef IMU_PROCESSOR_H
#define IMU_PROCESSOR_H

#include <vector>
#include <deque>
#include <cmath>
#include <iostream>
#include <limits>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <Arduino.h>

#define G_CONSTANT 9.81  // m/s²
#define IMPACT_THRESHOLD_LOW 2.5
#define IMPACT_THRESHOLD_MEDIUM 5.0
#define IMPACT_THRESHOLD_HIGH 7.5
#define IMPACT_THRESHOLD_SEVERE 10.0

struct IMUData {
    uint32_t timestamp;
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
};

class IMUProcessor {
public:
    static IMUProcessor& getInstance() {
        if (instance == nullptr) {
            instance = new IMUProcessor();
        }
        return *instance;
    }

    void processData(float accX, float accY, float accZ, 
                    float gyroX, float gyroY, float gyroZ,
                    uint32_t timestamp);
    void clearData();
    
    // Impact metrics calculation methods
    int getImpactLevel();
    double getHIC(double window_ms = 15.0);
    double getBrIC(double crit_x = 66.3, double crit_y = 66.3, double crit_z = 66.3);
    double getPeakLinearAcc();
    double getRidingVelocitybeforeImpact();
    double getHeadVelocityOnImpact();
    
private:
    static IMUProcessor* instance;
    std::deque<IMUData> imuDataBuffer;
    const size_t MAX_BUFFER_SIZE = 500;
    bool impactDetected = false;
    uint32_t lastImpactTime = 0;
    const uint32_t IMPACT_COOLDOWN = 2000;

    double calculateLinearAcceleration(const IMUData& data);
    double calculateAngularVelocity(const IMUData& data);
    std::vector<IMUData> getImpactWindow(uint32_t impactTime, double window_ms);
};

#endif