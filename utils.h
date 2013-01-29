#ifndef UTILS_H
#define UTILS_H

#include <iostream>

// forward declarations
class ArArgumentParser;
class ArRobot;

void echo(const std::string &msg);
void echo(const std::string &id, const int value);
void echo(const std::string &id, const double value);
void echo(const std::string &id, const std::string &value);
int getIndexForArg(int n, char **a, const char *argName);
void getCommandLineArguments(ArArgumentParser *parser,
    			     int argc, char **argv,
			     int *tilt, int *maxRange, int *minRange);
void testLaser(ArRobot *robot);

// distinguish invalid values
const int INVALID = -11;


#endif
