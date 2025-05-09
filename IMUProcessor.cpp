#include "IMUProcessor.hpp"
#include <algorithm>

IMUProcessor* IMUProcessor::instance = nullptr;

void IMUProcessor::processData(float accX, float accY, float accZ,
                               float gyroX, float gyroY, float gyroZ,
                               uint32_t timestamp) {
    IMUData processedData = { timestamp, accX, accY, accZ, gyroX, gyroY, gyroZ };
    imuDataBuffer.push_back(processedData);
    if (imuDataBuffer.size() > MAX_BUFFER_SIZE) {
        imuDataBuffer.pop_front();
    }
}

void IMUProcessor::clearData() {
    imuDataBuffer.clear();
    impactDetected = false;
    lastImpactTime = 0;
}

int IMUProcessor::getImpactLevel() {
    if (imuDataBuffer.empty()) return 0;
    
    // Check if we're in cooldown period
    if (impactDetected && (imuDataBuffer.back().timestamp - lastImpactTime < IMPACT_COOLDOWN)) {
        return 0;
    }
    
    double peakAcc = getPeakLinearAcc();
    
    if (peakAcc >= IMPACT_THRESHOLD_SEVERE) {
        impactDetected = true;
        lastImpactTime = imuDataBuffer.back().timestamp;
        return 4;
    } else if (peakAcc >= IMPACT_THRESHOLD_HIGH) {
        impactDetected = true;
        lastImpactTime = imuDataBuffer.back().timestamp;
        return 3;
    } else if (peakAcc >= IMPACT_THRESHOLD_MEDIUM) {
        impactDetected = true;
        lastImpactTime = imuDataBuffer.back().timestamp;
        return 2;
    } else if (peakAcc >= IMPACT_THRESHOLD_LOW) {
        impactDetected = true;
        lastImpactTime = imuDataBuffer.back().timestamp;
        return 1;
    }
    
    return 0;
}

double IMUProcessor::getHIC(double window_ms) {
    if (imuDataBuffer.empty() || !impactDetected) return 0.0;
    
    std::vector<IMUData> window = getImpactWindow(lastImpactTime, window_ms);
    if (window.empty()) return 0.0;
    
    double maxHIC = 0.0;
    double dt = window_ms / window.size();
    
    for (size_t i = 0; i < window.size(); ++i) {
        double sum = 0.0;
        for (size_t j = i; j < window.size(); ++j) {
            double acc = calculateLinearAcceleration(window[j]) / G_CONSTANT; // Convert to g
            sum += acc * dt;
            double t = (j - i + 1) * dt / 1000.0; // Convert to seconds
            double hic = pow(sum, 2.5) * t;
            maxHIC = std::max(maxHIC, hic);
        }
    }
    
    return maxHIC;
}

double IMUProcessor::getBrIC(double crit_x, double crit_y, double crit_z) {
    if (imuDataBuffer.empty() || !impactDetected) return 0.0;
    
    std::vector<IMUData> window = getImpactWindow(lastImpactTime, 15.0); // Use 15ms window for BrIC
    if (window.empty()) return 0.0;
    
    double max_omega_x = 0.0, max_omega_y = 0.0, max_omega_z = 0.0;
    
    for (const auto& data : window) {
        max_omega_x = std::max(max_omega_x, static_cast<double>(data.gyroX > 0 ? data.gyroX : -data.gyroX));
        max_omega_y = std::max(max_omega_y, static_cast<double>(data.gyroY > 0 ? data.gyroY : -data.gyroY));
        max_omega_z = std::max(max_omega_z, static_cast<double>(data.gyroZ > 0 ? data.gyroZ : -data.gyroZ));
    }
    
    double bric = sqrt(pow(max_omega_x/crit_x, 2) + 
                      pow(max_omega_y/crit_y, 2) + 
                      pow(max_omega_z/crit_z, 2));
    
    return bric;
}

double IMUProcessor::getPeakLinearAcc() {
    if (imuDataBuffer.empty()) return 0.0;
    
    double peakAcc = 0.0;
    for (const auto& data : imuDataBuffer) {
        peakAcc = std::max(peakAcc, calculateLinearAcceleration(data) / G_CONSTANT); // Convert to g
    }
    return peakAcc;
}

double IMUProcessor::getVelocityBeforeImpact() {
    if (imuDataBuffer.empty() || !impactDetected) return 0.0;
    
    // Get data from 100ms before impact
    std::vector<IMUData> window = getImpactWindow(lastImpactTime - 100, 100);
    if (window.empty()) return 0.0;
    
    return integrateAcceleration(window);
}

double IMUProcessor::getVelocityAfterImpact() {
    if (imuDataBuffer.empty() || !impactDetected) return 0.0;
    
    // Get data from 100ms after impact
    std::vector<IMUData> window = getImpactWindow(lastImpactTime, 100);
    if (window.empty()) return 0.0;
    
    return integrateAcceleration(window);
}

double IMUProcessor::calculateLinearAcceleration(const IMUData& data) {
    return sqrt(data.accX * data.accX + data.accY * data.accY + data.accZ * data.accZ);
}

double IMUProcessor::calculateAngularVelocity(const IMUData& data) {
    return sqrt(data.gyroX * data.gyroX + data.gyroY * data.gyroY + data.gyroZ * data.gyroZ);
}

std::vector<IMUData> IMUProcessor::getImpactWindow(uint32_t impactTime, double window_ms) {
    std::vector<IMUData> window;
    if (imuDataBuffer.empty()) return window;
    
    uint32_t startTime = impactTime - window_ms;
    uint32_t endTime = impactTime + window_ms;
    
    for (const auto& data : imuDataBuffer) {
        if (data.timestamp >= startTime && data.timestamp <= endTime) {
            window.push_back(data);
        }
    }
    
    return window;
}

double IMUProcessor::integrateAcceleration(const std::vector<IMUData>& window) {
    if (window.size() < 2) return 0.0;
    
    double velocity = 0.0;
    for (size_t i = 1; i < window.size(); ++i) {
        double dt = (window[i].timestamp - window[i-1].timestamp) / 1000.0; // Convert to seconds
        double acc = calculateLinearAcceleration(window[i-1]);
        velocity += acc * dt;
    }
    
    return velocity;
}
