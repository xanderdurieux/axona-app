#include "IMUProcessor.hpp"

IMUProcessor* IMUProcessor::instance = nullptr;

void IMUProcessor::clearData() {
    imuDataBuffer.clear();
    impactDetected = false;
    lastImpactTime = 0;
    biasCalculated = false;
    orientation = Quaternion(); // Reset orientation
    biasAccX = biasAccY = biasAccZ = 0.0f;
    biasGyroX = biasGyroY = biasGyroZ = 0.0f;
}


void IMUProcessor::processData(float accX, float accY, float accZ, 
                             float gyroX, float gyroY, float gyroZ,
                             uint32_t timestamp) {
    IMUData data;
    data.timestamp = timestamp;
    data.accX = accX;
    data.accY = accY;
    data.accZ = accZ;
    data.gyroX = gyroX;
    data.gyroY = gyroY;
    data.gyroZ = gyroZ;
    
    // Calculate time delta
    float dt = 0.0f;
    if (!imuDataBuffer.empty()) {
        dt = (timestamp - imuDataBuffer.back().timestamp) / 1000.0f; // Convert to seconds
    }
    
    // Update orientation using gyroscope data
    updateOrientation(data, dt);
    
    // Get gravity vector in sensor frame
    float gx = 0.0f, gy = 0.0f, gz = G_CONSTANT;
    rotateGravity(orientation, gx, gy, gz);
    
    // Calculate velocity components using trapezoidal integration
    if (!imuDataBuffer.empty()) {
        const IMUData& prev = imuDataBuffer.back();
        
        // Calculate acceleration components for both current and previous sample
        float currAccX = data.accX - biasAccX - gx;
        float currAccY = data.accY - biasAccY - gy;
        float currAccZ = data.accZ - biasAccZ - gz;
        
        float prevAccX = prev.accX - biasAccX - gx;
        float prevAccY = prev.accY - biasAccY - gy;
        float prevAccZ = prev.accZ - biasAccZ - gz;
        
        // Trapezoidal integration
        data.velX = prev.velX + (currAccX + prevAccX) * dt / 2.0f;
        data.velY = prev.velY + (currAccY + prevAccY) * dt / 2.0f;
        data.velZ = prev.velZ + (currAccZ + prevAccZ) * dt / 2.0f;
    } else {
        data.velX = data.velY = data.velZ = 0.0f;
    }
    
    // Update bias if not calculated yet
    if (!biasCalculated) {
        updateBias(data);
    }
    
    // Add to buffer
    imuDataBuffer.push_back(data);
    if (imuDataBuffer.size() > MAX_BUFFER_SIZE) {
        imuDataBuffer.pop_front();
    }

    // Check for impact
    if (biasCalculated) {
        double linearAcc = calculateLinearAcceleration(data);
        if (linearAcc > IMPACT_THRESHOLD_LOW && 
            (timestamp - lastImpactTime) > IMPACT_COOLDOWN) {
            impactDetected = true;
            lastImpactTime = timestamp;
        }
    }
}

Quaternion IMUProcessor::estimateOrientationFromAccel(const IMUData& data) {
    // Normalize acceleration vector
    float ax = data.accX - biasAccX;
    float ay = data.accY - biasAccY;
    float az = data.accZ - biasAccZ;
    float norm = sqrt(ax*ax + ay*ay + az*az);
    
    // Only use accelerometer if the magnitude is close to 1g
    if (abs(norm - G_CONSTANT) > 0.5) {
        return Quaternion(); // Return identity quaternion if acceleration is not reliable
    }
    
    ax /= norm;
    ay /= norm;
    az /= norm;
    
    // Calculate roll and pitch from accelerometer
    float roll = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay*ay + az*az));
    
    // Convert to quaternion
    float cy = cos(pitch * 0.5);
    float sy = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    
    return Quaternion(
        cy * cr,  // w
        cy * sr,  // x
        sy * cr,  // y
        sy * sr   // z
    );
}

void IMUProcessor::updateOrientation(const IMUData& data, float dt) {
    // Gyroscope-based orientation update
    Quaternion qDot(
        -0.5 * (data.gyroX * orientation.x + data.gyroY * orientation.y + data.gyroZ * orientation.z),
        0.5 * (data.gyroX * orientation.w + data.gyroZ * orientation.y - data.gyroY * orientation.z),
        0.5 * (data.gyroY * orientation.w - data.gyroZ * orientation.x + data.gyroX * orientation.z),
        0.5 * (data.gyroZ * orientation.w + data.gyroY * orientation.x - data.gyroX * orientation.y)
    );
    
    // Update orientation quaternion using the derivative
    Quaternion qGyro;
    qGyro.w = orientation.w + qDot.w * dt;
    qGyro.x = orientation.x + qDot.x * dt;
    qGyro.y = orientation.y + qDot.y * dt;
    qGyro.z = orientation.z + qDot.z * dt;
    qGyro.normalize();
    
    // Get accelerometer-based orientation
    Quaternion qAccel = estimateOrientationFromAccel(data);
    
    // Complementary filter
    // Use higher weight for gyroscope when there's significant motion
    float alpha = 0.96f;  // Gyroscope weight
    float accelMagnitude = sqrt(
        (data.accX - biasAccX) * (data.accX - biasAccX) +
        (data.accY - biasAccY) * (data.accY - biasAccY) +
        (data.accZ - biasAccZ) * (data.accZ - biasAccZ)
    );
    
    // Reduce gyroscope weight when acceleration is close to 1g
    if (abs(accelMagnitude - G_CONSTANT) < 0.5) {
        alpha = 0.8f;  // Give more weight to accelerometer when stable
    }
    
    // Combine gyroscope and accelerometer data
    orientation.w = alpha * qGyro.w + (1 - alpha) * qAccel.w;
    orientation.x = alpha * qGyro.x + (1 - alpha) * qAccel.x;
    orientation.y = alpha * qGyro.y + (1 - alpha) * qAccel.y;
    orientation.z = alpha * qGyro.z + (1 - alpha) * qAccel.z;
    
    // Normalize to prevent drift
    orientation.normalize();
}

void IMUProcessor::rotateGravity(const Quaternion& q, float& gx, float& gy, float& gz) {
    // Rotate gravity vector using quaternion
    float gx_orig = gx, gy_orig = gy, gz_orig = gz;
    
    gx = (1 - 2*q.y*q.y - 2*q.z*q.z) * gx_orig +
         (2*q.x*q.y - 2*q.w*q.z) * gy_orig +
         (2*q.x*q.z + 2*q.w*q.y) * gz_orig;
         
    gy = (2*q.x*q.y + 2*q.w*q.z) * gx_orig +
         (1 - 2*q.x*q.x - 2*q.z*q.z) * gy_orig +
         (2*q.y*q.z - 2*q.w*q.x) * gz_orig;
         
    gz = (2*q.x*q.z - 2*q.w*q.y) * gx_orig +
         (2*q.y*q.z + 2*q.w*q.x) * gy_orig +
         (1 - 2*q.x*q.x - 2*q.y*q.y) * gz_orig;
}

void IMUProcessor::updateBias(const IMUData& data) {
    static int sampleCount = 0;
    
    biasAccX += data.accX;
    biasAccY += data.accY;
    biasAccZ += data.accZ;
    biasGyroX += data.gyroX;
    biasGyroY += data.gyroY;
    biasGyroZ += data.gyroZ;
    
    sampleCount++;
    
    if (sampleCount >= BIAS_CALIBRATION_SAMPLES) {
        biasAccX /= BIAS_CALIBRATION_SAMPLES;
        biasAccY /= BIAS_CALIBRATION_SAMPLES;
        biasAccZ /= BIAS_CALIBRATION_SAMPLES;
        biasGyroX /= BIAS_CALIBRATION_SAMPLES;
        biasGyroY /= BIAS_CALIBRATION_SAMPLES;
        biasGyroZ /= BIAS_CALIBRATION_SAMPLES;
        
        // Adjust Z bias to account for gravity
        biasAccZ -= G_CONSTANT;
        
        biasCalculated = true;
    }
}

double IMUProcessor::calculateLinearAcceleration(const IMUData& data) {
    // Calculate linear acceleration by removing gravity and bias
    float ax = data.accX - biasAccX;
    float ay = data.accY - biasAccY;
    float az = data.accZ - biasAccZ;
    
    // Get gravity vector in sensor frame
    float gx = 0.0f, gy = 0.0f, gz = G_CONSTANT;
    rotateGravity(orientation, gx, gy, gz);
    
    // Remove gravity
    ax -= gx;
    ay -= gy;
    az -= gz;
    
    return sqrt(ax*ax + ay*ay + az*az);
}

double IMUProcessor::calculateAngularVelocity(const IMUData& data) {
    float wx = data.gyroX - biasGyroX;
    float wy = data.gyroY - biasGyroY;
    float wz = data.gyroZ - biasGyroZ;
    return sqrt(wx*wx + wy*wy + wz*wz);
}

double IMUProcessor::calculateVelocity(const IMUData& data) {
    return sqrt(data.velX*data.velX + data.velY*data.velY + data.velZ*data.velZ);
}

std::vector<IMUData> IMUProcessor::getImpactWindow(uint32_t impactTime, double window_ms) {
    std::vector<IMUData> window;
    uint32_t startTime = impactTime - static_cast<uint32_t>(window_ms);
    
    for (const auto& data : imuDataBuffer) {
        if (data.timestamp >= startTime && data.timestamp <= impactTime) {
            window.push_back(data);
        }
    }
    
    return window;
}

int IMUProcessor::getImpactLevel() {
    if (imuDataBuffer.empty()) return 0;
    
    double linearAcc = calculateLinearAcceleration(imuDataBuffer.back());
    
    if (linearAcc >= IMPACT_THRESHOLD_SEVERE) return 4;
    if (linearAcc >= IMPACT_THRESHOLD_HIGH) return 3;
    if (linearAcc >= IMPACT_THRESHOLD_MEDIUM) return 2;
    if (linearAcc >= IMPACT_THRESHOLD_LOW) return 1;
    return 0;
}

double IMUProcessor::getHIC(double window_ms) {
    if (imuDataBuffer.empty()) return 0.0;
    
    auto window = getImpactWindow(imuDataBuffer.back().timestamp, window_ms);
    if (window.empty()) return 0.0;
    
    double maxHIC = 0.0;
    double window_s = window_ms / 1000.0;  // Convert to seconds
    
    for (size_t i = 0; i < window.size(); ++i) {
        for (size_t j = i + 1; j < window.size(); ++j) {
            double dt = (window[j].timestamp - window[i].timestamp) / 1000.0;  // Convert to seconds
            if (dt > window_s) break;
            
            // Calculate average acceleration over the interval
            double sumAcc = 0.0;
            for (size_t k = i; k <= j; ++k) {
                // Convert linear acceleration to g's
                sumAcc += calculateLinearAcceleration(window[k]) / G_CONSTANT;
            }
            double avgAcc = sumAcc / (j - i + 1);
            
            // Calculate HIC using the standard formula
            double hic = dt * pow(avgAcc, 2.5);
            maxHIC = std::max(maxHIC, hic);
        }
    }
    
    return maxHIC;
}

double IMUProcessor::getAccOnImpact() {
    if (imuDataBuffer.empty()) return 0.0;
    if (!impactDetected) return 0.0;
    
    for (const auto& data : imuDataBuffer) {
        if (data.timestamp == lastImpactTime) {
            return calculateLinearAcceleration(data);
        }
    }
    return 0.0;
}

double IMUProcessor::getRidingVelocitybeforeImpact() {
    if (imuDataBuffer.empty()) return 0.0;
    
    // Get average velocity magnitude of 5s to 1s before impact
    std::vector<IMUData> velocityWindow = getImpactWindow(lastImpactTime - 5000, 4000);
    if (velocityWindow.empty()) return 0.0;
    
    double sumVelocity = 0.0;
    for (const auto& data : velocityWindow) {
        sumVelocity += calculateVelocity(data);
    }
    return sumVelocity / velocityWindow.size();
}

double IMUProcessor::getHeadVelocityOnImpact() {
    if (imuDataBuffer.empty()) return 0.0;
    
    // Get average velocity of 100ms before impact
    std::vector<IMUData> velocityWindow = getImpactWindow(lastImpactTime - 100, 100);
    if (velocityWindow.empty()) return 0.0;
    
    double sumVelocity = 0.0;
    for (const auto& data : velocityWindow) {
        sumVelocity += calculateVelocity(data);
    }
    return sumVelocity / velocityWindow.size();
}
