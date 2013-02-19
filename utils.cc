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

// get the configuration file name
const char *getConfigFile(ArArgumentParser &parser)
{
  const char *fileName = NULL;

  if (parser.checkParameterArgumentString("file", &fileName) &&
      fileName != NULL) {
    return fileName;
  }
  return NULL;
}

// get robot configuration information from configuration file
void configureRobot(ArArgumentParser &parser,
    		    int *tilt,
    		    A3dpoint &laserToRobotTranslation,
		    int *maxRange,
		    int *minRange)
{
  const int leftMargin = 5;
  const char *configArgs[] = {
    "tilt",
    "laserToRobotTranslationX",
    "laserToRobotTranslationY",
    "laserToRobotTranslationZ",
    "maxRange",
    "minRange"
  };

  std::cout << "Custom configuration options" << std::endl;

  int val = INVALID;
  // tilt value
  if (parser.checkParameterArgumentInteger(configArgs[0], &val) &&
      val != INVALID) {
    *tilt = val;
    std::cout << std::setw(leftMargin) << "| ";
    std::cout << "tilt = " << val << " degrees" << std::endl;
  }

  // laser to robot translation values
  val = INVALID;
  if (parser.checkParameterArgumentInteger(configArgs[1], &val) &&
      val != INVALID) {
    laserToRobotTranslation.x = val;
    std::cout << std::setw(leftMargin) << "| ";
    std::cout << "laserToRobotTranslationX = " << val 
      << " mm" << std::endl;
  }
  val = INVALID;
  if (parser.checkParameterArgumentInteger(configArgs[2], &val) &&
      val != INVALID) {
    laserToRobotTranslation.y = val;
    std::cout << std::setw(leftMargin) << "| ";
    std::cout << "laserToRobotTranslationY = " << val 
      << " mm" << std::endl;
  }
  val = INVALID;
  if (parser.checkParameterArgumentInteger(configArgs[3], &val) &&
      val != INVALID) {
    laserToRobotTranslation.z = val;
    std::cout << std::setw(leftMargin) << "| ";
    std::cout << "laserToRobotTranslationZ = " << val 
      << " mm" << std::endl;
  }

  // laser range values
  val = INVALID;
  if (parser.checkParameterArgumentInteger(configArgs[4], &val) &&
      val != INVALID) {
    *maxRange = val;
    std::cout << std::setw(leftMargin) << "| ";
    std::cout << "maxRange" << val << " mm" << std::endl;
  }
  val = INVALID;
  if (parser.checkParameterArgumentInteger(configArgs[5], &val) &&
      val != INVALID) {
    *minRange = val;
    std::cout << std::setw(leftMargin) << "| ";
    std::cout << "minRange" << val << " mm" << std::endl;
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
