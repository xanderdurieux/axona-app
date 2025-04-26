#include "IMUProcessor.hpp"

IMUProcessor::IMUProcessor() {}


void IMUProcessor::processData(float accX, float accY, float accZ,
                               float gyroX, float gyroY, float gyroZ,
                               uint32_t timestamp) {
    IMUData processedData = {timestamp, accX, accY, accZ, gyroX, gyroY, gyroZ};

    imuDataBuffer.push_back(processedData);
    
    while (imuDataBuffer.size() > 50) {
        imuDataBuffer.pop_front();
    }
}

int IMUProcessor::calculateImpactLevel() const {
    if (imuDataBuffer.size() < 5) {
        return -1;
    }
    
    double maxAcceleration = 0.0;
    
    for (const auto& data : imuDataBuffer) {
        double acceleration = data.getAcceleration() / 9.81; 
        
        if (acceleration > maxAcceleration) {
            maxAcceleration = acceleration;
        }
    }

    Serial.print("Acceleration: ");
    Serial.print(maxAcceleration);
    
    if (maxAcceleration < IMPACT_THRESHOLD_LOW) {
        return 0;
    } else if (maxAcceleration < IMPACT_THRESHOLD_MEDIUM) {
        return 1; 
    } else if (maxAcceleration < IMPACT_THRESHOLD_HIGH) {
        return 2; 
    } else if (maxAcceleration < IMPACT_THRESHOLD_SEVERE) {
        return 3; 
    } else {
        return 4;
    }
}

