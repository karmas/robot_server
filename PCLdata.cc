#include <map>
#include <ctime>
#include <sys/time.h>

#include "PCLdata.h"


const double PCLdata::toRadian = pi/180;


// sets the laser pointer now to the first laser in the robot
// also sets maxrange and minrange to laser max and zero if the
// values are invalid
PCLdata::PCLdata(ArServerBase *server, ArRobot *robot, int tilt,
                 const A3dpoint &laserToRobotTranslation,
                 int maxRange, int minRange)
  : myServer(server), myRobot(robot), myTilt(tilt),
    myLaserToRobotTranslation(laserToRobotTranslation),
    myMaxRange(maxRange), myMinRange(minRange),
    pclftr(this, &PCLdata::getData), myLaser(NULL)
{
  std::map<int, ArLaser *> *laserMap = myRobot->getLaserMap();
  std::map<int, ArLaser *>::iterator it;

  for (it = laserMap->begin(); it != laserMap->end(); it++) {
    myLaser = it->second;
    break;
  }

  if (myLaser != NULL) {
    if (myMaxRange == INVALID) myMaxRange = myLaser->getMaxRange();
    if (myMinRange == INVALID) myMinRange = 0;
  }

  // needed when server is not running laser
  if (myLaser == NULL)
    myLaser = new ArSick;

  echo("max laser range", myMaxRange);
  echo("min laser range", myMinRange);
  echo("units for laser", myLaser->getUnitsChoice());
}

// The first time this function is run, a starting time is set.
// Further calls return the time elapsed from that start time.
// This is necessary to get smaller values for time so that
// millisecond precision can be packet into the same data.
// Hence first packet is marked with time value of 0.
long PCLdata::getElapsedTime()
{
  static timeval startTime;
  timeval currTime;
  static bool firstTime = true;

  // set the start time
  if (firstTime) {
    firstTime = false;
    gettimeofday(&startTime, NULL);
    return 0;
  }
  else {
    gettimeofday(&currTime, NULL);
    long secondsPassed = currTime.tv_sec - startTime.tv_sec;
    // first get milliseconds
    long milliSecondsPassed = currTime.tv_usec/1000;
    // add the seconds passed to it
    milliSecondsPassed += secondsPassed*1000;
    return milliSecondsPassed;
  }
}


/* @param serverClient: Connection manager between the server and the 
 * 	client. It is provided by the Aria framework and is used to
 * 	transmit a packet to client.
 * @param packet: It is received from client. It does not exist for
 * 	this request.
 * @func: Converts laser readings into 3d co-ordinates and stores them
 * 	in a packet which is sent to the client. The packet format is:
 * 	TIME STAMP
 * 	ROBOT X CO-ORDINATE (DOUBLE)
 * 	ROBOT Y CO-ORDINATE (DOUBLE)
 * 	ROBOT HEADING (DOUBLE measured in degrees)
 * 	NUMBER OF READINGS (4 BYTES)
 * 	X CO-ORDINATE (DOUBLE)
 * 	Y CO-ORDINATE (DOUBLE)
 * 	Z CO-ORDINATE (DOUBLE)
 * 	X CO-ORDINATE (DOUBLE)
 * 	Y CO-ORDINATE (DOUBLE)
 * 	Z CO-ORDINATE (DOUBLE)
 * 	...
 *
 * Need to translate laser based co-ordinates to robot frame before
 * rotating using robot heading
 */
void PCLdata::getData(ArServerClient *serverClient, ArNetPacket *packet)
{
  const std::list<ArSensorReading *> * readings = NULL;
  std::list<ArSensorReading *>::const_iterator it;
  const ArSensorReading *reading = NULL;

  double distance = 0.0;
  double rawX = 0.0;
  double rawY = 0.0;
  double rawZ = 0.0;

  double localX = 0.0;
  double localY = 0.0;
  double localZ = 0.0;
  double globalX = 0.0;
  double globalY = 0.0;
  // angle between local x-axis and global x-axis
  double alpha = 0.0;
  // angle of laser reading
  double theta = 0.0;

  int readingRange = 0;
  std::vector<A3dpoint> points;

  ArNetPacket pclPacket;
  // Time stamp the packet
  long timeStamp = getElapsedTime();
  pclPacket.byte4ToBuf(timeStamp);

  // Fill robot location and heading
  pclPacket.doubleToBuf(myRobot->getX());
  pclPacket.doubleToBuf(myRobot->getY());
  pclPacket.doubleToBuf(myRobot->getTh());

  // construct a list of valid 3d co-ordinates for the laser readings
  readings = myLaser->getRawReadings();
  // check each reading from laser
  for (it = readings->begin(); it != readings->end(); it++) {
    reading = (*it);
    readingRange = reading->getRange();

    // only use valid readings
    if (!reading->getIgnoreThisReading() &&	// valid reading
	readingRange <= myMaxRange &&	// upper limit
	readingRange >= myMinRange) {	// lower limit
      // the angle made by the reading
      theta = reading->getSensorTh() * toRadian;
      // the distance to the point
      distance = reading->getRange();

      // tilted laser as reference frame
      rawX = distance * cos(theta);
      rawY = distance * sin(theta);
      rawZ = 0.0;

      // rotate on z-axis to account for the tilt
      // now the laser reference frame is parallel with robot's
      localX = rawX * cos(myTilt*toRadian);
      localY = rawY;
      localZ = rawX * sin(myTilt*toRadian);

      // translate to robot reference frame
      localX += myLaserToRobotTranslation.x;
      localY += myLaserToRobotTranslation.y;
      localZ += myLaserToRobotTranslation.z;

      // rotate to global reference frame
      alpha = reading->getThTaken() * toRadian;
      globalX = localX*cos(alpha) - localY*sin(alpha);
      globalY = localY*cos(alpha) + localX*sin(alpha);

      // translate to global reference frame
      globalX += myRobot->getX();
      globalY += myRobot->getY();

      // remember the valid points
      points.push_back(A3dpoint(globalX, globalY, localZ));
    }
  }

  // add number of readings to packet
  pclPacket.byte4ToBuf(points.size());

  // add all the points to the packet
  for (size_t i = 0; i < points.size(); i++) {
    pclPacket.doubleToBuf(points[i].x);
    pclPacket.doubleToBuf(points[i].y);
    pclPacket.doubleToBuf(points[i].z);
  }

  // send the packet to the client
  serverClient->sendPacketTcp(&pclPacket);
}
