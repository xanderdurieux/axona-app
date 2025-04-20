#include "BLEManager.h"

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
  int length = characteristic.valueLength();
  if (length < 2) return; // Minimum length for Movesense packets

  const uint8_t* data = characteristic.value();
  uint8_t packet_type = data[0];

  static uint8_t ongoing_data[256]; // Buffer for ongoing data
  static int ongoing_data_length = 0;

  if (packet_type == DATA) {
    // First part of the data
    memcpy(ongoing_data, data + 2, length - 2); // Skip packet type and reference
    ongoing_data_length = length - 2;
    return;
  }

  if (packet_type == DATA_PART2 || packet_type == DATA_PART3) {
    // Append subsequent parts
    memcpy(ongoing_data + ongoing_data_length, data + 2, length - 2);
    ongoing_data_length += length - 2;

    if (packet_type == DATA_PART3) {
      // Final part, process the combined data
      const uint8_t* combined_data = ongoing_data;
      uint32_t timestamp;
      memcpy(&timestamp, &combined_data[0], sizeof(uint32_t));

      const uint8_t* sensor_data = &combined_data[4]; // Start after timestamp
      const int row_count = 8;
      const int row_stride = 3 * sizeof(float);
      const int block_stride = row_count * row_stride;

      for (int i = 0; i < row_count; ++i) {
        uint32_t row_timestamp = timestamp + int(i * 1000 / 104);

        IMUData imu;
        imu.timestamp = row_timestamp;

        // Acc: 0..block_stride
        memcpy(&imu.accX,  &sensor_data[i * row_stride + 0], sizeof(float));
        memcpy(&imu.accY,  &sensor_data[i * row_stride + 4], sizeof(float));
        memcpy(&imu.accZ,  &sensor_data[i * row_stride + 8], sizeof(float));

        // Gyro: offset by one block
        memcpy(&imu.gyroX, &sensor_data[block_stride + i * row_stride + 0], sizeof(float));
        memcpy(&imu.gyroY, &sensor_data[block_stride + i * row_stride + 4], sizeof(float));
        memcpy(&imu.gyroZ, &sensor_data[block_stride + i * row_stride + 8], sizeof(float));

        // Mag: offset by two blocks
        memcpy(&imu.magX,  &sensor_data[2 * block_stride + i * row_stride + 0], sizeof(float));
        memcpy(&imu.magY,  &sensor_data[2 * block_stride + i * row_stride + 4], sizeof(float));
        memcpy(&imu.magZ,  &sensor_data[2 * block_stride + i * row_stride + 8], sizeof(float));

        // Use the singleton IMUProcessor instance
        IMUProcessor::getInstance().processIMUData(imu);
      }

      // Reset buffer
      ongoing_data_length = 0;
    }
  }
}
