#include <iomanip>
#include <list>

#include "Aria.h"
#include "ArNetworking.h"

#include "utils.h"


// some message display routines
void echo(const std::string &msg)
{
  std::cout << "\t" << msg << std::endl;
}
void echo(const std::string &id, const int value)
{
  std::cout << "\t" << id << " = " << value << std::endl;
}
void echo(const std::string &id, const double value)
{
  std::cout << "\t" << id << " = " << value << std::endl;
}
void echo(const std::string &id, const std::string &value)
{
  std::cout << "\t" << id << " = " << value << std::endl;
}


// returns index in argv of the value for given argName
// returns INVALID if value not found or argument not found
int getIndexForArg(int n, char **a, const char *argName)
{
  int index = INVALID;
  for (int i = 1; i < n; i++) {
    if (strcmp(a[i], argName) == 0) {
      index = i+1;
      break;
    }
  }

  if (index < n) return index;
  else return INVALID;
}

// populate given parameters with values from the command line
// if they are available
void getCommandLineArguments(ArArgumentParser *parser,
    			     int argc, char **argv,
			     int *tilt, int *maxRange, int *minRange)
{
  int index;
  const char *tiltArg = "-tilt";
  const char *maxRangeArg = "-maxRange";
  const char *minRangeArg = "-minRange";

  // print options
  std::cout << "Here are the following options" << std::endl;
  std::cout << "\t" << tiltArg << " in degrees" <<  std::endl;
  std::cout << "\t" << maxRangeArg << " in mm" << std::endl;
  std::cout << "\t" << minRangeArg << " in mm" << std::endl;

  // try to find tilt index
  if (parser->checkArgument(tiltArg)) {
    index = getIndexForArg(argc, argv, tiltArg);
    if (index != -1) {
      *tilt = atoi(argv[index]);
      std::cout << "laser tilt = " << *tilt << " degrees" << std::endl;
    }
  }

  // try to find max range
  if (parser->checkArgument(maxRangeArg)) {
    index = getIndexForArg(argc, argv, maxRangeArg);
    if (index != -1) {
      *maxRange = atoi(argv[index]);
      std::cout << "max range for laser is " << maxRange 
	<< " mm" << std::endl;
    }
    else {
      echo("you did not give value for max range");
    }
  }

  // try to find min range
  if (parser->checkArgument(minRangeArg)) {
    index = getIndexForArg(argc, argv, minRangeArg);
    if (index != -1) {
      *minRange = atoi(argv[index]);
      std::cout << "min range for laser is " << maxRange 
	<< " mm" << std::endl;
    }
    else {
      echo("you did not give value for min range");
    }
  }
}

// laser testing routine
void testLaser(ArRobot *robot)
{
  ArLaser *laser = robot->findLaser(1);

  const std::list<ArSensorReading *> * readings = NULL;
  std::list<ArSensorReading *>::const_iterator it;
  ArSensorReading *reading = NULL;

  const int WIDTH = 8;
  const std::string SEP = "||";

  std::cout << "\t LASER READINGS" << std::endl;
  std::cout << std::setw(WIDTH) << "localX" << SEP;
  std::cout << std::setw(WIDTH) << "localY" << SEP;
  std::cout << std::setw(WIDTH) << "range" << SEP;
  std::cout << std::setw(WIDTH) << "sensorTh" << SEP;
  std::cout << std::setw(WIDTH) << "robotTh" << SEP;
  std::cout << std::endl;

  int i = 0;
  readings = laser->getRawReadings();
  for (it = readings->begin(); it != readings->end(); it++) {
    ++i;
    if (i % 10 == 0) {
      reading = (*it);
      // check if the laser reading is valid
      if (!reading->getIgnoreThisReading()) {
	std::cout << std::setw(WIDTH) << reading->getLocalX() << SEP;
	std::cout << std::setw(WIDTH) << reading->getLocalY() << SEP;
	std::cout << std::setw(WIDTH) << reading->getRange() << SEP;
	std::cout <<std::setw(WIDTH)<< reading->getSensorTh() << SEP;
	std::cout <<std::setw(WIDTH)<< reading->getThTaken() << SEP;
	std::cout << std::endl;
      }
    }
  }
  std::cout << std::endl;
}
