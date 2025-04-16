#ifndef COMMAND_PROCESSOR_H
#define COMMAND_PROCESSOR_H

#include <Arduino.h>
#include "BLEManager.h"

class CommandProcessor;

struct Command {
  const char* name;
  const char* description;
  const char* usage;
  bool (CommandProcessor::*handler)(int argc, char** argv) ;
};

class CommandProcessor {
public:
  CommandProcessor(BLEManager &manager) : bleManager(manager) {};
  void processInput();
  void printHelp();

private:  
  static const Command COMMANDS[];
  static const int COMMAND_COUNT;
  
  BLEManager &bleManager;
  
  bool helpHandler(int argc, char** argv);
  bool scanHandler(int argc, char** argv);
  bool listHandler(int argc, char** argv);
  bool selectHandler(int argc, char** argv);
  bool servicesHandler(int argc, char** argv);
  bool subscribeHandler(int argc, char** argv);
  bool readHandler(int argc, char** argv);
  bool writeHandler(int argc, char** argv);
  bool disconnectHandler(int argc, char** argv);
  bool movesenseHandler(int argc, char** argv);

  // Helper methods
  char** tokenizeInput(String input, int& argc);
  void freeTokens(char** argv, int argc);
};

#endif
