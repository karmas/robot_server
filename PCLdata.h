#ifndef PCLDATA_H
#define PCLDATA_H

#include "Aria.h"
#include "ArNetworking.h"

// used to pack x,y,z values together
struct A3dpoint {
  A3dpoint(double v1, double v2, double v3)
    : x(v1), y(v2), z(v3) {}
  double x;
  double y;
  double z;
};


// This class houses the method which is called when a request packet
// is of type "getPCL". 
class PCLdata {
public:
  PCLdata(ArServerBase *server, ArRobot *robot, int tilt,
          int maxRange, int minRange);
  void getData(ArServerClient *serverClient, ArNetPacket *packet);
  long getElapsedTime();

  // members for pcl data
  static const double pi = 3.14159165f;
  static const double toRadian;

  ArServerBase *myServer;
  ArRobot *myRobot;
  int myTilt;
  int myMaxRange;
  int myMinRange;

  ArFunctor2C<PCLdata, ArServerClient *, ArNetPacket *> pclftr;
  ArLaser *myLaser;
};


#endif
