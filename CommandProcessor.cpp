#include "CommandProcessor.hpp"

CommandProcessor::CommandProcessor(BLEManager* bleManager)
  : bleManager(bleManager) {}

const CommandProcessor::Command CommandProcessor::COMMANDS[] = {
  {"help", "Show available commands", "help", &CommandProcessor::helpHandler},
  {"scan", "Scan for BLE devices", "scan", &CommandProcessor::scanHandler},
  {"list", "List scanned devices", "list", &CommandProcessor::listHandler},
  {"select", "Select a device to connect", "select <device index>", &CommandProcessor::selectHandler},
  {"services", "List services and characteristics", "services", &CommandProcessor::servicesHandler},
  {"subscribe", "Subscribe to a characteristic", "subscribe <service index> <characteristic index>", &CommandProcessor::subscribeHandler},
  {"unsubscribe", "Unsubscribe from a characteristic", "unsubscribe <service index> <characteristic index>", &CommandProcessor::unsubscribeHandler},
  {"read", "Read from a characteristic", "read <service index> <characteristic index>", &CommandProcessor::readHandler},
  {"write", "Write to a characteristic", "write <service index> <characteristic index> <hex data>", &CommandProcessor::writeHandler},
  {"disconnect", "Disconnect from the device", "disconnect", &CommandProcessor::disconnectHandler},
  {"movesense", "Send Movesense command", "movesense <service index>", &CommandProcessor::movesenseHandler},
  {"auto", "Automatically connect and subscribe to Movesense", "auto", &CommandProcessor::autoHandler}
};

const int CommandProcessor::COMMAND_COUNT = sizeof(COMMANDS) / sizeof(COMMANDS[0]);

void CommandProcessor::processInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) {
      return;
    }

    int argc = 0;
    char** argv = tokenizeInput(input, argc);
    
    if (argc > 0) {
      bool commandFound = false;
      for (int i = 0; i < COMMAND_COUNT; i++) {
        if (strcmp(argv[0], COMMANDS[i].name) == 0) {
          commandFound = true;
          if (!(this->*COMMANDS[i].handler)(argc - 1, &argv[1])) {
            Serial.print("Usage: ");
            Serial.println(COMMANDS[i].usage);
          }
          break;
        }
      }
      
      if (!commandFound) {
        Serial.println("Unknown command. Type 'help' for available commands.");
      }
    }
    
    freeTokens(argv, argc);
  }
}

void CommandProcessor::printHelp() {
  Serial.println("Available Commands:");
  Serial.println("==================");
  
  for (int i = 0; i < COMMAND_COUNT; i++) {
    Serial.print("\n");
    Serial.print(COMMANDS[i].usage);
    Serial.print("\n    ");
    Serial.println(COMMANDS[i].description);
  }
  Serial.println();
}

char** CommandProcessor::tokenizeInput(String input, int& argc) {
  int maxTokens = 1;
  for (unsigned int i = 0; i < input.length(); i++) {
    if (input.charAt(i) == ' ') maxTokens++;
  }
    char** argv = new char*[maxTokens];
  argc = 0;
  char* inputCopy = new char[input.length() + 1];
  strcpy(inputCopy, input.c_str());
  
  char* token = strtok(inputCopy, " ");
  while (token != nullptr && argc < maxTokens) {
    argv[argc] = new char[strlen(token) + 1];
    strcpy(argv[argc], token);
    argc++;
    token = strtok(nullptr, " ");
  }
  
  delete[] inputCopy;
  return argv;
}

void CommandProcessor::freeTokens(char** argv, int argc) {
  for (int i = 0; i < argc; i++) {
    delete[] argv[i];
  }
  delete[] argv;
}

bool CommandProcessor::helpHandler(int argc, char** argv) {
  printHelp();
  return true;
}

bool CommandProcessor::scanHandler(int argc, char** argv) {
  bleManager->scanDevices();
  return true;
}

bool CommandProcessor::listHandler(int argc, char** argv) {
  bleManager->listDevices();
  return true;
}

bool CommandProcessor::selectHandler(int argc, char** argv) {
  if (argc != 1) return false;
  int index = atoi(argv[0]);
  bleManager->selectDevice(index);
  return true;
}

bool CommandProcessor::servicesHandler(int argc, char** argv) {
  bleManager->listServicesAndCharacteristics();
  return true;
}

bool CommandProcessor::subscribeHandler(int argc, char** argv) {
  if (argc != 2) return false;
  int sIndex = atoi(argv[0]);
  int cIndex = atoi(argv[1]);
  bleManager->subscribeCharacteristic(sIndex, cIndex);
  return true;
}

bool CommandProcessor::unsubscribeHandler(int argc, char** argv) {
  if (argc != 2) return false;
  int sIndex = atoi(argv[0]);
  int cIndex = atoi(argv[1]);
  bleManager->unsubscribeCharacteristic(sIndex, cIndex);
  return true;
}

bool CommandProcessor::readHandler(int argc, char** argv) {
  if (argc != 2) return false;
  int sIndex = atoi(argv[0]);
  int cIndex = atoi(argv[1]);
  bleManager->readCharacteristic(sIndex, cIndex);
  return true;
}

bool CommandProcessor::writeHandler(int argc, char** argv) {
  if (argc < 3) return false;
  
  int sIndex = atoi(argv[0]);
  int cIndex = atoi(argv[1]);
  
  uint8_t* data = new uint8_t[argc - 2];
  for (int i = 2; i < argc; i++) {
    data[i - 2] = static_cast<uint8_t>(atoi(argv[i]));
  }
  
  bleManager->writeCharacteristic(sIndex, cIndex, data, sizeof(data));
  
  return true;
}

bool CommandProcessor::disconnectHandler(int argc, char** argv) {
  bleManager->disconnect();
  return true;
}

bool CommandProcessor::movesenseHandler(int argc, char** argv) {
  if (argc < 1) return false;
  
  String subcommand = argv[0];

  int serviceIndex = 5;
  int notifyCharIndex = 1;
  int writeCharIndex = 0; 
  
  if (subcommand == "hello") {
    const uint8_t helloMessage[] = {0, 123};
    bleManager->subscribeCharacteristic(serviceIndex, notifyCharIndex);
    bleManager->writeCharacteristic(serviceIndex, writeCharIndex, helloMessage, sizeof(helloMessage));
    Serial.println("Sent hello command to Movesense");
  }
  else if (subcommand == "subscribe") {
    String sampleRate = argv[1];
    const uint8_t subscribeCommand[] = {1, 99, '/', 'M', 'e', 'a', 's', '/', 'I', 'M', 'U', '6', '/', '5', '2'};
    bleManager->writeCharacteristic(serviceIndex, writeCharIndex, subscribeCommand, sizeof(subscribeCommand));
    bleManager->subscribeCharacteristic(serviceIndex, notifyCharIndex);
    Serial.println("Subscribed to IMU sensor");
  }
  else if (subcommand == "unsubscribe") {
    const uint8_t unsubscribeCommand[] = {2, 99};
    bleManager->writeCharacteristic(serviceIndex, writeCharIndex, unsubscribeCommand, sizeof(unsubscribeCommand));
    bleManager->unsubscribeCharacteristic(serviceIndex, notifyCharIndex);
    Serial.println("Unsubscribed from IMU sensor");
  }
  else {
    Serial.print("Unknown movesense command: ");
    Serial.println(subcommand);
    return false;
  }

}

bool CommandProcessor::autoHandler(int argc, char** argv) {
  // Execute "scan"
  if (!scanHandler(0, nullptr)) {
    Serial.println("Failed to scan for devices.");
    return false;
  }

  // Find and select the device with the specific address
  const char* targetAddress = "74:92:ba:10:e8:23";
  int targetIndex = bleManager->getDeviceIndex(targetAddress);
  if (targetIndex == -1) {
    Serial.println("Target device not found.");
    return false;

  }

  char targetIndexStr[4];
  snprintf(targetIndexStr, sizeof(targetIndexStr), "%d", targetIndex);
  const char* selectArgs[] = {targetIndexStr};

  // Execute "select <device_index>"
  if (!selectHandler(1, const_cast<char**>(selectArgs))) {
    Serial.println("Failed to select device.");
    return false;
  }

  // Execute "services"
  if (!servicesHandler(0, nullptr)) {
    Serial.println("Failed to list services.");
    return false;
  }

  // Execute "movesense subscribe"
  const char* movesenseArgs[] = {"subscribe", "52"};
  if (!movesenseHandler(1, const_cast<char**>(movesenseArgs))) {
    Serial.println("Failed to subscribe to Movesense.");
    return false;
  }

  Serial.println("Auto command executed successfully.");
  return true;
}