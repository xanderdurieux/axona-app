#include "BLEManager.hpp"

// #define BLE_DEBUG

/**
 * @brief Initialize the BLE module
 * 
 * @return true if BLE initialization was successful
 * @return false if BLE initialization failed
 */
bool BLEManager::begin() {
  bool success = BLE.begin();
#ifdef BLE_DEBUG
  if (success) {
    Serial.println("BLE initialized");
  } else {
    Serial.println("Failed to initialize BLE!");
  }
#endif
  return success;
}

/**
 * @brief Process BLE events
 * 
 * This method should be called regularly in the main loop
 */
void BLEManager::poll() {
  BLE.poll();
}

/**
 * @brief Check if a device is already in the scanned devices list
 * 
 * @param device The BLE device to check
 * @return true if the device is already in the list
 * @return false if the device is not in the list
 */
bool BLEManager::deviceAlreadyListed(BLEDevice device) {
  for (int i = 0; i < deviceCount; i++) {
    if (scannedDevices[i].address() == device.address()) {
      return true;
    }
  }
  return false;
}

/**
 * @brief Scan for available BLE devices
 * 
 * Scans for BLE devices for the duration specified by SCAN_TIME
 * and adds them to the scannedDevices array if they meet the
 * signal strength requirements (MIN_RSSI)
 */
void BLEManager::scanDevices() {
  deviceCount = 0;
#ifdef BLE_DEBUG
  Serial.println("Scanning for BLE devices...");
#endif
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
#ifdef BLE_DEBUG
  Serial.println("Scan complete.\n");
#endif
}

/**
 * @brief Get the index of a device in the scannedDevices array by address
 * 
 * @param address The BLE address to search for
 * @return int The index of the device, or -1 if not found
 */
int BLEManager::getDeviceIndex(String address) {
  for (int i = 0; i < deviceCount; i++) {
    if (scannedDevices[i].address() == address) {
      return i;
    }
  }
  return -1;
}

/**
 * @brief List all discovered devices to the Serial console
 * 
 * Displays device index, address, and name (if available)
 */
void BLEManager::listDevices() {
  if (deviceCount == 0) {
#ifdef BLE_DEBUG
    Serial.println("No devices found. Run the 'scan' command first.");
#endif
    return;
  }
  #ifdef BLE_DEBUG
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
  #endif
}

/**
 * @brief Connect to a device by its index in the scannedDevices array
 * 
 * @param index The index of the device to connect to
 * @return true if connection was successful
 * @return false if connection failed
 */
bool BLEManager::selectDevice(int index) {
  if (index < 0 || index >= deviceCount) {
#ifdef BLE_DEBUG
    Serial.println("Invalid device index.");
#endif
    return false;
  }
#ifdef BLE_DEBUG
  Serial.print("Connecting to device ");
  Serial.println(scannedDevices[index].address());
#endif
  selectedDevice = scannedDevices[index];
  if (selectedDevice.connect()) {
#ifdef BLE_DEBUG
    Serial.println("Connected successfully!");
#endif
    return true;
  } else {
#ifdef BLE_DEBUG
    Serial.println("Failed to connect.");
#endif
    selectedDevice = BLEDevice(); // Clear selection
    return false;
  }
}

/**
 * @brief Discover and list all services and characteristics of the connected device
 * 
 * Lists all services and characteristics to the Serial console, showing UUIDs
 * and properties (read, write, notify)
 */
void BLEManager::listServicesAndCharacteristics() {
  if (!selectedDevice) {
#ifdef BLE_DEBUG
    Serial.println("No device connected.");
#endif
    return;
  }
#ifdef BLE_DEBUG
  Serial.println("Discovering services...");
#endif
  if (!selectedDevice.discoverAttributes()) {
#ifdef BLE_DEBUG
    Serial.println("Failed to discover attributes.");
#endif
    return;
  }
  
  int serviceCount = selectedDevice.serviceCount();
  if (serviceCount == 0) {
#ifdef BLE_DEBUG
    Serial.println("No services found.");
#endif
    return;
  }
  
#ifdef BLE_DEBUG
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
#endif
}

/**
 * @brief Subscribe to notifications from a characteristic
 * 
 * @param sIndex The index of the service
 * @param cIndex The index of the characteristic
 * @return true if subscription was successful
 * @return false if subscription failed
 */
bool BLEManager::subscribeCharacteristic(int sIndex, int cIndex) {
  if (!selectedDevice) {
#ifdef BLE_DEBUG
    Serial.println("No device connected.");
#endif
    return false;
  }
  if (!selectedDevice.discoverAttributes()) {
#ifdef BLE_DEBUG
      Serial.println("Service discovery failed.");
#endif
    return false;
  }
  int svcCount = selectedDevice.serviceCount();
  if (sIndex < 0 || sIndex >= svcCount) {
#ifdef BLE_DEBUG
    Serial.println("Invalid service index.");
#endif
    return false;
  }
  BLEService service = selectedDevice.service(sIndex);
  int charCount = service.characteristicCount();
  if (cIndex < 0 || cIndex >= charCount) {
#ifdef BLE_DEBUG
    Serial.println("Invalid characteristic index.");
#endif
    return false;
  }
  selectedCharacteristic = service.characteristic(cIndex);
  if (selectedCharacteristic.canSubscribe()) {
    if (selectedCharacteristic.subscribe()) {
      selectedCharacteristic.setEventHandler(BLEUpdated, notificationCallback);
#ifdef BLE_DEBUG
      Serial.print("Subscribed to characteristic ");
      Serial.println(selectedCharacteristic.uuid());
#endif
      subscribed = true;
      return true;
    } else {
#ifdef BLE_DEBUG
      Serial.println("Subscription failed.");
#endif
      return false;
    }
  } else {
#ifdef BLE_DEBUG
    Serial.println("Characteristic does not support notifications.");
#endif
    return false;
  }
}

/**
 * @brief Unsubscribe from notifications from a characteristic
 * 
 * @param sIndex The index of the service
 * @param cIndex The index of the characteristic
 * @return true if unsubscription was successful
 * @return false if unsubscription failed
 */
bool BLEManager::unsubscribeCharacteristic(int sIndex, int cIndex) {
  if (!selectedDevice) {
#ifdef BLE_DEBUG
    Serial.println("No device connected.");
#endif
    return false;
  }
  if (!selectedDevice.discoverAttributes()) {
#ifdef BLE_DEBUG
    Serial.println("Service discovery failed.");
#endif
    return false;
  }
  int svcCount = selectedDevice.serviceCount();
  if (sIndex < 0 || sIndex >= svcCount) {
#ifdef BLE_DEBUG
    Serial.println("Invalid service index.");
#endif
    return false;
  }
  BLEService service = selectedDevice.service(sIndex);
  int charCount = service.characteristicCount();
  if (cIndex < 0 || cIndex >= charCount) {
#ifdef BLE_DEBUG
    Serial.println("Invalid characteristic index.");
#endif
    return false;
  }
  BLECharacteristic characteristic = service.characteristic(cIndex);
  if (characteristic.canSubscribe()) {
    if (characteristic.unsubscribe()) {
#ifdef BLE_DEBUG
      Serial.print("Unsubscribed from characteristic ");
      Serial.println(characteristic.uuid());
#endif
      subscribed = false;
      IMUProcessor::getInstance().clearData();
      return true;
    } else {
#ifdef BLE_DEBUG
      Serial.println("Unsubscribe failed.");
#endif
      return false;
    }
  } else {
#ifdef BLE_DEBUG
    Serial.println("Characteristic does not support notifications.");
#endif
    return false;
  }
}

/**
 * @brief Read the value of a characteristic
 * 
 * @param sIndex The index of the service
 * @param cIndex The index of the characteristic
 * @return true if read was successful
 * @return false if read failed
 */
bool BLEManager::readCharacteristic(int sIndex, int cIndex) {
  if (!selectedDevice) {
#ifdef BLE_DEBUG
    Serial.println("No device connected.");
#endif
    return false;
  }
  if (!selectedDevice.discoverAttributes()) {
#ifdef BLE_DEBUG
    Serial.println("Service discovery failed.");
#endif
    return false;
  }
  int svcCount = selectedDevice.serviceCount();
  if (sIndex < 0 || sIndex >= svcCount) {
#ifdef BLE_DEBUG
    Serial.println("Invalid service index.");
#endif
    return false;
  }
  BLEService service = selectedDevice.service(sIndex);
  int charCount = service.characteristicCount();
  if (cIndex < 0 || cIndex >= charCount) {
#ifdef BLE_DEBUG
    Serial.println("Invalid characteristic index.");
#endif
    return false;
  }
  BLECharacteristic characteristic = service.characteristic(cIndex);
  if (characteristic.canRead()) {
    if (characteristic.read()) {
#ifdef BLE_DEBUG
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
#endif
      return true;
    } else {
#ifdef BLE_DEBUG
      Serial.println("Read failed.");
#endif
      return false;
    }
  } else {
#ifdef BLE_DEBUG
    Serial.println("Characteristic is not readable.");
#endif
    return false;
  }
}

/**
 * @brief Write a value to a characteristic
 * 
 * @param sIndex The index of the service
 * @param cIndex The index of the characteristic
 * @param data Pointer to the data to write
 * @param length Length of the data to write
 * @return true if write was successful
 * @return false if write failed
 */
bool BLEManager::writeCharacteristic(int sIndex, int cIndex, const uint8_t *data, int length) {
  if (!selectedDevice) {
#ifdef BLE_DEBUG
    Serial.println("No device connected.");
#endif
    return false;
  }
  if (!selectedDevice.discoverAttributes()) {
#ifdef BLE_DEBUG
    Serial.println("Service discovery failed.");
#endif
    return false;
  }
  int svcCount = selectedDevice.serviceCount();
  if (sIndex < 0 || sIndex >= svcCount) {
#ifdef BLE_DEBUG
    Serial.println("Invalid service index.");
#endif
    return false;
  }
  BLEService service = selectedDevice.service(sIndex);
  int charCount = service.characteristicCount();
  if (cIndex < 0 || cIndex >= charCount) {
#ifdef BLE_DEBUG
    Serial.println("Invalid characteristic index.");
#endif
    return false;
  }
  BLECharacteristic characteristic = service.characteristic(cIndex);
  if (characteristic.canWrite()) {
    if (characteristic.writeValue(data, length)) {
#ifdef BLE_DEBUG
      Serial.print("Wrote ");
      for (int i = 0; i < length; i++) {
        Serial.print(data[i], HEX);
        Serial.print(" ");
      }
      Serial.print("to characteristic ");
      Serial.print(characteristic.uuid());
      Serial.println(".");
#endif
      return true;
    } else {
#ifdef BLE_DEBUG
      Serial.println("Write failed.");
#endif
      return false;
    }
  } else {
#ifdef BLE_DEBUG
    Serial.println("Characteristic is not writable.");
#endif
    return false;
  }
}

/**
 * @brief Disconnect from the currently connected device
 * 
 * Closes the connection with the currently selected device
 */
void BLEManager::disconnect() {
  if (selectedDevice) {
    selectedDevice.disconnect();
#ifdef BLE_DEBUG
    Serial.println("Disconnected.");
#endif
    selectedDevice = BLEDevice();
#ifdef BLE_DEBUG
  } else {
    Serial.println("No device is currently connected.");
#endif
  }
}

/**
 * @brief Callback function for BLE characteristic notifications
 * 
 * This method is called when a subscribed characteristic is updated.
 * It processes the received IMU data and forwards it to the IMUProcessor.
 * 
 * @param device The BLE device that sent the notification
 * @param characteristic The characteristic that was updated
 */
void BLEManager::notificationCallback(BLEDevice device, BLECharacteristic characteristic) {
  int length = characteristic.valueLength();
  const uint8_t* data = characteristic.value();

  if (length < 6) {
#ifdef BLE_DEBUG
    Serial.println("Data too short.");
#endif
    return;
  }
  if (length > 150) {
#ifdef BLE_DEBUG
    Serial.println("Data length exceeds maximum limit.");
#endif
    return;
  }

  DataView dv = DataView(data, length);

  uint8_t packet_type = dv.getUint8(0);
  uint16_t reference = dv.getUint8(1);
  uint32_t timestamp = dv.getUint32(2);

  const int sensorDataSize = 12;
  const int numRows = (length - 2) / (2 * sensorDataSize); 
  const int sampleRate = numRows * 13;
  
  IMUProcessor& processor = IMUProcessor::getInstance();

// #ifdef BLE_DEBUG
//   Serial.print("IMU Data - Timestamp: ");
//   Serial.print(timestamp);
//   Serial.print(" Sample Rate: ");
//   Serial.print(sampleRate);
//   Serial.print(" Hz\n");
// #endif

  for (int i = 0; i < numRows; ++i) {
    uint32_t row_timestamp = timestamp + int(i * 1000 / sampleRate);

    float accX = dv.getFloat32(6 + i * sensorDataSize);
    float accY = dv.getFloat32(6 + i * sensorDataSize + 4);
    float accZ = dv.getFloat32(6 + i * sensorDataSize + 8);

    float gyroX = dv.getFloat32(6 + numRows * sensorDataSize + i * sensorDataSize);
    float gyroY = dv.getFloat32(6 + numRows * sensorDataSize + i * sensorDataSize + 4);
    float gyroZ = dv.getFloat32(6 + numRows * sensorDataSize + i * sensorDataSize + 8);

#ifdef BLE_DEBUG
    // Format for serial plotter
    Serial.print("accX:");
    Serial.print(accX);
    Serial.print(" accY:");
    Serial.print(accY);
    Serial.print(" accZ:");
    Serial.print(accZ);
    Serial.print(" gyroX:");
    Serial.print(gyroX);
    Serial.print(" gyroY:");
    Serial.print(gyroY);
    Serial.print(" gyroZ:");
    Serial.println(gyroZ);
#endif

    processor.processData(accX, accY, accZ, gyroX, gyroY, gyroZ, row_timestamp);
  }
}
