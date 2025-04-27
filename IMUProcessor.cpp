#include "IMUProcessor.hpp"
#include <algorithm>

IMUProcessor::IMUProcessor() {}

void IMUProcessor::processData(float accX, float accY, float accZ,
                               float gyroX, float gyroY, float gyroZ,
                               uint32_t timestamp) {
    IMUData processedData = { timestamp, accX, accY, accZ, gyroX, gyroY, gyroZ };
    imuDataBuffer.push_back(processedData);
    if (imuDataBuffer.size() > 50) {
        imuDataBuffer.pop_front();
    }
}

void IMUProcessor::clearData() {
    imuDataBuffer.clear();
}

int IMUProcessor::calculateImpactLevel() const {
    if (imuDataBuffer.size() < 5) return -1;
    double maxG = 0.0;
    for (const auto& data : imuDataBuffer) {
        double g = data.getAcceleration() / 9.81;
        maxG = std::max(maxG, g);
    }
    // thresholds assumed defined elsewhere
    if (maxG < IMPACT_THRESHOLD_LOW)     return 0;
    else if (maxG < IMPACT_THRESHOLD_MEDIUM)  return 1;
    else if (maxG < IMPACT_THRESHOLD_HIGH)    return 2;
    else if (maxG < IMPACT_THRESHOLD_SEVERE)  return 3;
    else                                     return 4;
}

// Peak linear acceleration (m/s²)
double IMUProcessor::getPeakLinearAcc() const {
    double peak = 0;
    for (const auto& m : imuDataBuffer) {
        peak = std::max(peak, static_cast<double>(m.getAcceleration()));
    }
    return peak;
}

// Gadd Severity Index: SI = ∫ a(t)^2.5 dt
double IMUProcessor::getGaddSI() const {
    if (imuDataBuffer.size() < 2) return 0;
    double SI = 0;
    for (size_t i = 1; i < imuDataBuffer.size(); ++i) {
        const auto& prev = imuDataBuffer[i-1];
        const auto& cur  = imuDataBuffer[i];
        double dt = (cur.timestamp - prev.timestamp) / 1000.0;
        double a  = cur.getAcceleration();
        SI += std::pow(a, 2.5) * dt;
    }
    return SI;
}

// Head Injury Criterion (HIC)
double IMUProcessor::getHIC(double windowMs) const {
    if (imuDataBuffer.size() < 2) return 0;
    double maxHIC = 0;
    double maxW = windowMs / 1000.0;
    for (size_t i = 0; i < imuDataBuffer.size(); ++i) {
        double t0 = imuDataBuffer[i].timestamp / 1000.0;
        double area = 0;
        for (size_t j = i + 1; j < imuDataBuffer.size(); ++j) {
            double t1 = imuDataBuffer[j].timestamp / 1000.0;
            double dt = (imuDataBuffer[j].timestamp - imuDataBuffer[j-1].timestamp) / 1000.0;
            double T = t1 - t0;
            if (T > maxW) break;
            area += imuDataBuffer[j-1].getAcceleration() * dt;
            double avgA = area / T;
            double H = std::pow(avgA, 2.5) * T;
            maxHIC = std::max(maxHIC, H);
        }
    }
    return maxHIC;
}

// Peak angular acceleration (rad/s²)
double IMUProcessor::getPeakAngularAcc() const {
    if (imuDataBuffer.size() < 2) return 0;
    double peak = 0;
    for (size_t i = 1; i < imuDataBuffer.size(); ++i) {
        const auto& prev = imuDataBuffer[i-1];
        const auto& cur  = imuDataBuffer[i];
        double dt = (cur.timestamp - prev.timestamp) / 1000.0;
        if (dt <= 0) continue;
        double ax = (cur.gyroX - prev.gyroX) / dt;
        double ay = (cur.gyroY - prev.gyroY) / dt;
        double az = (cur.gyroZ - prev.gyroZ) / dt;
        double mag = std::sqrt(ax*ax + ay*ay + az*az);
        peak = std::max(peak, mag);
    }
    return peak;
}

// Brain Injury Criterion (BrIC)
double IMUProcessor::getBrIC(double omegaCX, double omegaCY, double omegaCZ) const {
    double px=0, py=0, pz=0;
    for (const auto& m : imuDataBuffer) {
        px = std::max(px, std::fabs(static_cast<double>(m.gyroX)));
        py = std::max(py, std::fabs(static_cast<double>(m.gyroY)));
        pz = std::max(pz, std::fabs(static_cast<double>(m.gyroZ)));
    }
    return std::sqrt(
        std::pow(px/omegaCX, 2) +
        std::pow(py/omegaCY, 2) +
        std::pow(pz/omegaCZ, 2)
    );
}

// Rotational Injury Criterion (RIC)
double IMUProcessor::getRIC(double windowMs) const {
    // build angular accel series
    std::vector<AngularSample> Adata;
    for (size_t i = 1; i < imuDataBuffer.size(); ++i) {
        const auto& prev = imuDataBuffer[i-1];
        const auto& cur  = imuDataBuffer[i];
        double dt = (cur.timestamp - prev.timestamp) / 1000.0;
        if (dt <= 0) continue;
        double ax = (cur.gyroX - prev.gyroX) / dt;
        double ay = (cur.gyroY - prev.gyroY) / dt;
        double az = (cur.gyroZ - prev.gyroZ) / dt;
        Adata.push_back({ cur.timestamp/1000.0, std::sqrt(ax*ax + ay*ay + az*az) });
    }
    if (Adata.size() < 2) return 0;
    double maxRIC = 0;
    double maxW = windowMs / 1000.0;
    for (size_t i = 0; i < Adata.size(); ++i) {
        double t0 = Adata[i].time, area=0;
        for (size_t j = i+1; j < Adata.size(); ++j) {
            double t1 = Adata[j].time;
            double dt = t1 - Adata[j-1].time;
            double T = t1 - t0;
            if (T > maxW) break;
            area += Adata[j-1].alpha * dt;
            double avg = area / T;
            double R = std::pow(avg, 2.5) * T;
            maxRIC = std::max(maxRIC, R);
        }
    }
    return maxRIC;
}
