#ifndef PCLDATA_H
#define PCLDATA_H

#include "Aria.h"
#include "ArNetworking.h"

#include "utils.h"


// forward declarations
class stereoCam;


// Abstract base class for robot sensor data
class SensorData {
public:
  SensorData(ArServerBase *server, ArRobot *robot);
  virtual ~SensorData() {}

  static const double pi;
  static const double toRadian;
protected:
  ArServerBase *myServer;
  ArRobot *myRobot;

  virtual void send(ArServerClient *serverClient, ArNetPacket *packet) = 0;
  virtual void addData() = 0;
};


// Sends laser data
class SensorDataLaser : public SensorData {
public:
  SensorDataLaser(ArServerBase *server, ArRobot *robot,
      		  int tilt, const A3dpoint &laserToRobotTranslation,
		  int maxRange, int minRange);
  virtual void send(ArServerClient *serverClient, ArNetPacket *packet);
  virtual void addData();

private:
  ArFunctor2C<SensorDataLaser, ArServerClient *, ArNetPacket *> mySendFtr;
  ArLaser *myLaser;
  int myTilt;
  A3dpoint myLaserToRobotTranslation;
  int myMaxRange;
  int myMinRange;
};


// Sends stereo camera data
class SensorDataStereoCam : public SensorData {
public:
  SensorDataStereoCam(ArServerBase *server, ArRobot *robot);
  virtual ~SensorDataStereoCam();
  virtual void send(ArServerClient *serverClient, ArNetPacket *packet);
  virtual void addData();

private:
  bool invalidPoint(double x, double y, double z);

  ArFunctor2C<SensorDataStereoCam, ArServerClient *, ArNetPacket *> 
    mySendFtr;
  ArDPPTU *myPTU;
  stereoCam *myCam;
};

#endif
