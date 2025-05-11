#ifndef COMMAND_PROCESSOR_H
#define COMMAND_PROCESSOR_H

#include <Arduino.h>
#include "BLEManager.hpp"

class CommandProcessor {
public:
  static CommandProcessor& getInstance(BLEManager* bleManager = nullptr) {
    static CommandProcessor instance(bleManager);
    return instance;
  }
  
  void processInput();
  void printHelp();

private:
  typedef bool (CommandProcessor::*CommandHandler)(int argc, char** argv);

  struct Command {
    const char* name;
    const char* description;
    const char* usage;
    CommandHandler handler;
  };

  CommandProcessor(BLEManager* bleManager);
  CommandProcessor(const CommandProcessor&) = delete;
  CommandProcessor& operator=(const CommandProcessor&) = delete;

  char** tokenizeInput(String input, int& argc);
  void freeTokens(char** argv, int argc);

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
  
  BLEManager* bleManager;
  static const Command COMMANDS[];
  static const int COMMAND_COUNT;
};

#endif
