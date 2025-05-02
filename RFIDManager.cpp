#include "RFIDManager.hpp"

RFIDManager::RFIDManager(uint8_t ssPin, uint8_t rstPin) 
    : rfid(ssPin, rstPin), ssPin(ssPin), rstPin(rstPin) {}

bool RFIDManager::begin() {
    SPI.begin();
    rfid.PCD_Init();
    bool success = rfid.PCD_PerformSelfTest();
		if (success) {
				Serial.println("RFID module initialized successfully.");
		} else {
				Serial.println("RFID module initialization failed.");
		}
		return success;		
}

bool RFIDManager::writeData(const String& data) {
    if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
        return false;
    }

    MFRC522::StatusCode status = rfid.MIFARE_Write(4, (byte*)data.c_str(), data.length());
    rfid.PICC_HaltA();
    return status == MFRC522::STATUS_OK;
}

String RFIDManager::readData() {
    if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
        return "";
    }

    byte buffer[18];
    byte size = sizeof(buffer);
    MFRC522::StatusCode status = rfid.MIFARE_Read(4, buffer, &size);
    rfid.PICC_HaltA();

    if (status != MFRC522::STATUS_OK) {
        return "";
    }

    return String((char*)buffer);
}
