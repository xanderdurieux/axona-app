#ifndef RFID_MANAGER_HPP
#define RFID_MANAGER_HPP

#include <MFRC522.h>
#include <SPI.h>

class RFIDManager {
public:
    RFIDManager(uint8_t ssPin, uint8_t rstPin);
    bool begin();
    bool writeData(const String& data);
    String readData();

private:
    MFRC522 rfid;
    uint8_t ssPin;
    uint8_t rstPin;
};

#endif
