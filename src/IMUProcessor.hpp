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

// Simple quaternion representation
struct Quaternion {
    double w, x, y, z;
    Quaternion(double w_=1, double x_=0, double y_=0, double z_=0)
        : w(w_), x(x_), y(y_), z(z_) {}
    
    // Multiply this quaternion by another
    Quaternion operator*(const Quaternion &o) const {
        return Quaternion(
            w*o.w - x*o.x - y*o.y - z*o.z,
            w*o.x + x*o.w + y*o.z - z*o.y,
            w*o.y - x*o.z + y*o.w + z*o.x,
            w*o.z + x*o.y - y*o.x + z*o.w
        );
    }
    
    // Normalize quaternion
    void normalize() {
        double n = std::sqrt(w*w + x*x + y*y + z*z);
        w /= n; x /= n; y /= n; z /= n;
    }
};

struct IMUData {
    uint32_t timestamp;
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float velX, velY, velZ;
};

class IMUProcessor {
public:
    static IMUProcessor& getInstance() {
        if (instance == nullptr) {
            instance = new IMUProcessor();
        }
        return *instance;
    }

    void clearData();
    void processData(float accX, float accY, float accZ, 
                    float gyroX, float gyroY, float gyroZ,
                    uint32_t timestamp);
    
    // Impact metrics calculation methods
    int getImpactLevel();
    double getHIC(double window_ms = 15.0);
    double getAccOnImpact();
    double getRidingVelocitybeforeImpact();
    double getHeadVelocityOnImpact();
    
private:
    static IMUProcessor* instance;
    std::deque<IMUData> imuDataBuffer;
    const size_t MAX_BUFFER_SIZE = 500;
    bool impactDetected = false;
    uint32_t lastImpactTime = 0;
    const uint32_t IMPACT_COOLDOWN = 2000;
    Quaternion orientation;
    
    // Bias calculation
    float biasAccX = 0.0f, biasAccY = 0.0f, biasAccZ = 0.0f;
    float biasGyroX = 0.0f, biasGyroY = 0.0f, biasGyroZ = 0.0f;
    bool biasCalculated = false;
    const int BIAS_CALIBRATION_SAMPLES = 50;

    // Helper methods
    void updateOrientation(const IMUData& data, float dt);
    void rotateGravity(const Quaternion& q, float& gx, float& gy, float& gz);
    void updateBias(const IMUData& data);
    double calculateLinearAcceleration(const IMUData& data);
    double calculateAngularVelocity(const IMUData& data);
    double calculateVelocity(const IMUData& data);
    std::vector<IMUData> getImpactWindow(uint32_t impactTime, double window_ms);
};

#endif