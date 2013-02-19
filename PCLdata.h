#ifndef PCLDATA_H
#define PCLDATA_H

#include "Aria.h"
#include "ArNetworking.h"

#include "utils.h"

// This class houses the method which is called when a request packet
// is of type "getPCL". 
class PCLdata {
public:
  PCLdata(ArServerBase *server, ArRobot *robot, int tilt,
      	  const A3dpoint &laserToRobotTranslation,
          int maxRange, int minRange);
  void getData(ArServerClient *serverClient, ArNetPacket *packet);
  long getElapsedTime();

  // members for pcl data
  static const double pi = 3.14159165f;
  static const double toRadian;

  ArServerBase *myServer;
  ArRobot *myRobot;
  int myTilt;
  A3dpoint myLaserToRobotTranslation;
  int myMaxRange;
  int myMinRange;

  ArFunctor2C<PCLdata, ArServerClient *, ArNetPacket *> pclftr;
  ArLaser *myLaser;
};


#endif
