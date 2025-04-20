#include "BLEManager.hpp"

bool BLEManager::deviceAlreadyListed(BLEDevice device) {
  for (int i = 0; i < deviceCount; i++) {
    if (scannedDevices[i].address() == device.address()) {
      return true;
    }
  }
  return false;
}

void BLEManager::scanDevices() {
  deviceCount = 0;
  Serial.println("Scanning for BLE devices...");
  BLE.scan();
  unsigned long startTime = millis();
  while (millis() - startTime < SCAN_TIME) {
    BLEDevice dev = BLE.available();
    if (dev) {
      if (!deviceAlreadyListed(dev) && deviceCount < MAX_DEVICES && dev.rssi() >= MIN_RSSI) {
        scannedDevices[deviceCount++] = dev;
      }
    }
  }
  BLE.stopScan();
  Serial.println("Scan complete.\n");
}

int BLEManager::getDeviceIndex(String address) {
  for (int i = 0; i < deviceCount; i++) {
    if (scannedDevices[i].address() == address) {
      return i;
    }
  }
  return -1;
}

void BLEManager::listDevices() {
  if (deviceCount == 0) {
    Serial.println("No devices found. Run the 'scan' command first.");
    return;
  }
  Serial.println("Available devices:");
  for (int i = 0; i < deviceCount; i++) {
    Serial.print("  [");
    Serial.print(i);
    Serial.print("] ");
    Serial.print(scannedDevices[i].address());
    if (scannedDevices[i].hasLocalName()) {
      Serial.print(" (");
      Serial.print(scannedDevices[i].localName());
      Serial.print(")");
    }

    Serial.println();
  }
  Serial.println("Use command 'select <device index>' to connect.\n");
}

bool BLEManager::selectDevice(int index) {
  if (index < 0 || index >= deviceCount) {
    Serial.println("Invalid device index.");
    return false;
  }
  Serial.print("Connecting to device ");
  Serial.println(scannedDevices[index].address());
  selectedDevice = scannedDevices[index];
  if (selectedDevice.connect()) {
    Serial.println("Connected successfully!");
    return true;
  } else {
    Serial.println("Failed to connect.");
    selectedDevice = BLEDevice(); // Clear selection
    return false;
  }
}

void BLEManager::listServicesAndCharacteristics() {
  if (!selectedDevice) {
    Serial.println("No device connected.");
    return;
  }
  Serial.println("Discovering services...");
  if (!selectedDevice.discoverAttributes()) {
    Serial.println("Failed to discover attributes.");
    return;
  }
  
  int serviceCount = selectedDevice.serviceCount();
  if (serviceCount == 0) {
    Serial.println("No services found.");
    return;
  }
  
  for (int i = 0; i < serviceCount; i++) {
    BLEService service = selectedDevice.service(i);
    Serial.print("Service [");
    Serial.print(i);
    Serial.print("] UUID: ");
    Serial.println(service.uuid());
    
    int charCount = service.characteristicCount();
    for (int j = 0; j < charCount; j++) {
      BLECharacteristic characteristic = service.characteristic(j);
      Serial.print("  Characteristic [");
      Serial.print(j);
      Serial.print("] UUID: ");
      Serial.print(characteristic.uuid());
      Serial.print("  ");
      if (characteristic.canRead()) Serial.print("R ");
      if (characteristic.canWrite()) Serial.print("W ");
      if (characteristic.canSubscribe()) Serial.print("N ");
      Serial.println();
    }
    Serial.println();
  }
  Serial.println("To subscribe, use: subscribe <service index> <characteristic index>");
  Serial.println("To read, use: read <service index> <characteristic index>");
  Serial.println("To write, use: write <service index> <characteristic index> <hex data>\n");
}

bool BLEManager::subscribeCharacteristic(int sIndex, int cIndex) {
  if (!selectedDevice) {
    Serial.println("No device connected.");
    return false;
  }
  if (!selectedDevice.discoverAttributes()) {
    Serial.println("Service discovery failed.");
    return false;
  }
  int svcCount = selectedDevice.serviceCount();
  if (sIndex < 0 || sIndex >= svcCount) {
    Serial.println("Invalid service index.");
    return false;
  }
  BLEService service = selectedDevice.service(sIndex);
  int charCount = service.characteristicCount();
  if (cIndex < 0 || cIndex >= charCount) {
    Serial.println("Invalid characteristic index.");
    return false;
  }
  selectedCharacteristic = service.characteristic(cIndex);
  if (selectedCharacteristic.canSubscribe()) {
    if (selectedCharacteristic.subscribe()) {
      selectedCharacteristic.setEventHandler(BLEUpdated, notificationCallback);
      Serial.print("Subscribed to characteristic ");
      Serial.println(selectedCharacteristic.uuid());
      isSubscribed = true;
      return true;
    } else {
      Serial.println("Subscription failed.");
      return false;
    }
  } else {
    Serial.println("Characteristic does not support notifications.");
    return false;
  }
}

bool BLEManager::unsubscribeCharacteristic(int sIndex, int cIndex) {
  if (!selectedDevice) {
    Serial.println("No device connected.");
    return false;
  }
  if (!selectedDevice.discoverAttributes()) {
    Serial.println("Service discovery failed.");
    return false;
  }
  int svcCount = selectedDevice.serviceCount();
  if (sIndex < 0 || sIndex >= svcCount) {
    Serial.println("Invalid service index.");
    return false;
  }
  BLEService service = selectedDevice.service(sIndex);
  int charCount = service.characteristicCount();
  if (cIndex < 0 || cIndex >= charCount) {
    Serial.println("Invalid characteristic index.");
    return false;
  }
  BLECharacteristic characteristic = service.characteristic(cIndex);
  if (characteristic.canSubscribe()) {
    if (characteristic.unsubscribe()) {
      Serial.print("Unsubscribed from characteristic ");
      Serial.println(characteristic.uuid());
      isSubscribed = false;
      return true;
    } else {
      Serial.println("Unsubscribe failed.");
      return false;
    }
  } else {
    Serial.println("Characteristic does not support notifications.");
    return false;
  }
}

bool BLEManager::readCharacteristic(int sIndex, int cIndex) {
  if (!selectedDevice) {
    Serial.println("No device connected.");
    return false;
  }
  if (!selectedDevice.discoverAttributes()) {
    Serial.println("Service discovery failed.");
    return false;
  }
  int svcCount = selectedDevice.serviceCount();
  if (sIndex < 0 || sIndex >= svcCount) {
    Serial.println("Invalid service index.");
    return false;
  }
  BLEService service = selectedDevice.service(sIndex);
  int charCount = service.characteristicCount();
  if (cIndex < 0 || cIndex >= charCount) {
    Serial.println("Invalid characteristic index.");
    return false;
  }
  BLECharacteristic characteristic = service.characteristic(cIndex);
  if (characteristic.canRead()) {
    if (characteristic.read()) {
      Serial.print("Read from ");
      Serial.print(characteristic.uuid());
      Serial.print(": ");
      int len = characteristic.valueLength();
      const uint8_t* data = characteristic.value();
      for (int i = 0; i < len; i++) {
        Serial.print(data[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      return true;
    } else {
      Serial.println("Read failed.");
      return false;
    }
  } else {
    Serial.println("Characteristic is not readable.");
    return false;
  }
}

bool BLEManager::writeCharacteristic(int sIndex, int cIndex, const uint8_t *data, int length) {
  if (!selectedDevice) {
    Serial.println("No device connected.");
    return false;
  }
  if (!selectedDevice.discoverAttributes()) {
    Serial.println("Service discovery failed.");
    return false;
  }
  int svcCount = selectedDevice.serviceCount();
  if (sIndex < 0 || sIndex >= svcCount) {
    Serial.println("Invalid service index.");
    return false;
  }
  BLEService service = selectedDevice.service(sIndex);
  int charCount = service.characteristicCount();
  if (cIndex < 0 || cIndex >= charCount) {
    Serial.println("Invalid characteristic index.");
    return false;
  }
  BLECharacteristic characteristic = service.characteristic(cIndex);
  if (characteristic.canWrite()) {
    if (characteristic.writeValue(data, length)) {
      Serial.print("Wrote ");
      for (int i = 0; i < length; i++) {
        Serial.print(data[i], HEX);
        Serial.print(" ");
      }
      Serial.print("to characteristic ");
      Serial.print(characteristic.uuid());
      Serial.println(".");
      return true;
    } else {
      Serial.println("Write failed.");
      return false;
    }
  } else {
    Serial.println("Characteristic is not writable.");
    return false;
  }
}

void BLEManager::disconnect() {
  if (selectedDevice) {
    selectedDevice.disconnect();
    Serial.println("Disconnected.");
    selectedDevice = BLEDevice();
  } else {
    Serial.println("No device is currently connected.");
  }
}

void BLEManager::notificationCallback(BLEDevice device, BLECharacteristic characteristic) {
  static int sampleCounter = 0;
  static float startTime = millis();
  static float sampleRate = 0;

  Serial.print(sampleCounter);
  Serial.print(" | Sample rate: ");
  Serial.print(10.0 / ((sampleRate) / 1000.0));
  Serial.println(" Hz");

  sampleCounter++;
  if (sampleCounter % 10 == 0) {
    sampleRate = millis() - startTime;
    startTime = millis();
  }

  static uint8_t* ongoingDataBuffer = nullptr;
  static size_t ongoingDataLength = 0;

  int length = characteristic.valueLength();
  if (length < 2) return;

  const uint8_t* data = characteristic.value();
  uint8_t packet_type = data[0];
  uint8_t reference = data[1];

  if (packet_type == DATA) {
    Serial.println("DATA PART 1");
    // Store the first part of incoming data
    if (ongoingDataBuffer != nullptr) {
      free(ongoingDataBuffer);
    }
    
    ongoingDataBuffer = (uint8_t*)malloc(length);
    if (ongoingDataBuffer) {
      memcpy(ongoingDataBuffer, data, length);
      ongoingDataLength = length;
    }
  } 
  else if (packet_type == DATA_PART2 && ongoingDataBuffer != nullptr) {
    Serial.println("DATA PART 2");
    // Create combined data buffer (skip packet type and reference from part2)
    size_t combinedLength = ongoingDataLength + length - 2;
    uint8_t* combinedData = (uint8_t*)malloc(combinedLength);
    
    if (combinedData) {
      memcpy(combinedData, ongoingDataBuffer, ongoingDataLength);
      memcpy(combinedData + ongoingDataLength, data + 2, length - 2);
      
      // Process the combined data
      uint32_t timestamp;
      memcpy(&timestamp, &combinedData[2], sizeof(uint32_t));

      const uint8_t* sensor_data = &combinedData[6]; // Start after packet type, reference and timestamp
      const int row_count = 8;
      const int row_stride = 3 * sizeof(float);
      const int block_stride = row_count * row_stride;

      for (int i = 0; i < row_count; ++i) {
        uint32_t row_timestamp = timestamp + int(i * 1000 / 104);
        
        // Directly extract values without using IMUData
        float accX, accY, accZ;
        float gyroX, gyroY, gyroZ;
        float magX, magY, magZ;
        
        int offset = i * row_stride;
        int skip = block_stride;
        
        // Accelerometer data
        memcpy(&accX, &sensor_data[offset], sizeof(float));
        memcpy(&accY, &sensor_data[offset + 4], sizeof(float));
        memcpy(&accZ, &sensor_data[offset + 8], sizeof(float));
        
        // Gyroscope data
        memcpy(&gyroX, &sensor_data[offset + skip], sizeof(float));
        memcpy(&gyroY, &sensor_data[offset + skip + 4], sizeof(float));
        memcpy(&gyroZ, &sensor_data[offset + skip + 8], sizeof(float));
        
        // Magnetometer data
        memcpy(&magX, &sensor_data[offset + 2 * skip], sizeof(float));
        memcpy(&magY, &sensor_data[offset + 2 * skip + 4], sizeof(float));
        memcpy(&magZ, &sensor_data[offset + 2 * skip + 8], sizeof(float));

        // Format and print the IMU data directly
        char buffer[128];
        sprintf(buffer, "IMU9,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
                row_timestamp,
                accX, accY, accZ,
                gyroX, gyroY, gyroZ,
                magX, magY, magZ);
        Serial.println(buffer);
      }
      
      free(combinedData);
    }
    
    // Clean up
    free(ongoingDataBuffer);
    ongoingDataBuffer = nullptr;
  }
}
