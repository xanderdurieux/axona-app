#include "IMUProcessor.h"


void IMUProcessor::processIMUData(const IMUData& imu) {
  downsampleAndStore(imu);
  if (bufferIndex == 0) { // Buffer full, process data
    float avgImpact = 0;
    for (int i = 0; i < 10; i++) {
      IMUData temp;
      temp.accX = buffer[i][0];
      temp.accY = buffer[i][1];
      temp.accZ = buffer[i][2];
      avgImpact += calculateImpact(temp);
    }
    avgImpact /= 10;
    Serial.print("Average Impact: ");
    Serial.println(avgImpact);
  }
}

float IMUProcessor::calculateImpact(const IMUData& imu) {
    // Calculate impact as the magnitude of acceleration
    return sqrt(imu.accX * imu.accX + imu.accY * imu.accY + imu.accZ * imu.accZ);
}

void IMUProcessor::downsampleAndStore(const IMUData& imu) {
  buffer[bufferIndex][0] = imu.accX;
  buffer[bufferIndex][1] = imu.accY;
  buffer[bufferIndex][2] = imu.accZ;
  buffer[bufferIndex][3] = imu.gyroX;
  buffer[bufferIndex][4] = imu.gyroY;
  buffer[bufferIndex][5] = imu.gyroZ;
  buffer[bufferIndex][6] = imu.magX;
  buffer[bufferIndex][7] = imu.magY;
  buffer[bufferIndex][8] = imu.magZ;
  bufferIndex = (bufferIndex + 1) % 10;
}

void IMUProcessor::resetBuffer() {
  memset(buffer, 0, sizeof(buffer));
  bufferIndex = 0;
}
