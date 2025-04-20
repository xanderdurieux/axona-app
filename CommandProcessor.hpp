#ifndef COMMAND_PROCESSOR_H
#define COMMAND_PROCESSOR_H

#include <Arduino.h>
#include <string.h>
#include "BLEManager.hpp"

class CommandProcessor;

struct Command {
  const char* name;
  const char* description;
  const char* usage;
  bool (CommandProcessor::*handler)(int argc, char** argv);
};

class CommandProcessor {
public:
  static CommandProcessor& getInstance(BLEManager* bleManager = nullptr) {
    static CommandProcessor instance(bleManager);
    return instance;
  }

  void processInput();
  void printHelp();

private:
  CommandProcessor(BLEManager* bleManager);
  CommandProcessor(const CommandProcessor&) = delete;
  CommandProcessor& operator=(const CommandProcessor&) = delete;

  BLEManager* bleManager;

  static const Command COMMANDS[];
  static const int COMMAND_COUNT;

  bool helpHandler(int argc, char** argv);
  bool scanHandler(int argc, char** argv);
  bool listHandler(int argc, char** argv);
  bool selectHandler(int argc, char** argv);
  bool servicesHandler(int argc, char** argv);
  bool subscribeHandler(int argc, char** argv);
  bool unsubscribeHandler(int argc, char** argv);
  bool readHandler(int argc, char** argv);
  bool writeHandler(int argc, char** argv);
  bool disconnectHandler(int argc, char** argv);
  bool movesenseHandler(int argc, char** argv);
  bool autoHandler(int argc, char** argv);

  // Helper methods
  char** tokenizeInput(String input, int& argc);
  void freeTokens(char** argv, int argc);
};

#endif
