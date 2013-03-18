#include <map>
#include <ctime>
#include <sys/time.h>

#include "SensorData.h"


SensorData::SensorData(ArServerBase *server, ArRobot *robot)
  : myServer(server), myRobot(robot)
{
}


const double SensorDataLaser::pi = 3.14159165f;
const double SensorDataLaser::toRadian = pi/180;


// sets the laser pointer now to the first laser in the robot
// also sets maxrange and minrange to laser max and zero if the
// values are invalid
SensorDataLaser::SensorDataLaser(ArServerBase *server, ArRobot *robot, 
    int tilt, const A3dpoint &laserToRobotTranslation,
    int maxRange, int minRange)
  : SensorData(server, robot),
    mySendFtr(this, &SensorDataLaser::send), 
    myLaser(NULL), myTilt(tilt),
    myLaserToRobotTranslation(laserToRobotTranslation),
    myMaxRange(maxRange), myMinRange(minRange)
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

  addData();
}


/* @param serverClient: Connection manager between the server and the 
 * 	client. It is provided by the Aria framework and is used to
 * 	transmit a packet to client.
 * @param packet: It is received from client. It does not exist for
 * 	this request.
 * @func: Converts laser readings into 3d co-ordinates and stores them
 * 	in a packet which is sent to the client. The packet format is:
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
void SensorDataLaser::send(ArServerClient *serverClient, ArNetPacket *packet)
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


// Adds laser data service to the server
void SensorDataLaser::addData()
{
  myServer->addData("getSensorDataLaser",// packet name
     		    "sends laser data",	// description
     		    &(this->mySendFtr),	// callback functor	
     		    "no arguments",	// description of arguments
     		   			// needed from client
     		    "sends a packet containing 3d co-ordinates");
}


SensorDataStereoCam::SensorDataStereoCam(ArServerBase *server, 
    ArRobot *robot)
  : SensorData(server, robot),
    mySendFtr(this, &SensorDataStereoCam::send)
{
  addData();
}


// prepares a packet consisting of data read from stereocamera
//
// -----------------
// IMAGE HEIGHT
// IMAGE WIDTH
// NUMBER OF CHANNELS IN EACH PIXEL
// PIXEL 1 X
// PIXEL 1 Y
// PIXEL 1 Z
// PIXEL 1 COLOR 1
// PIXEL 1 COLOR 2
// PIXEL 1 COLOR 3
// PIXEL 2 X
// PIXEL 2 Y
// PIXEL 2 Z
// PIXEL 2 COLOR 1
// PIXEL 2 COLOR 2
// PIXEL 2 COLOR 3
//   .
//   .
// -----------------
void SensorDataStereoCam::send(ArServerClient *serverClient, 
    ArNetPacket *packet)
{
  /*
  // The width and height take on specific values. Change with
  // caution. Other values may not work with doStereoFrame.
  static const int width = 320;
  static const int height = 240;
  IplImage *coordImg = 
    cvCreateImage(cvSize(width,height), IPL_DEPTH_64F, 3);
  IplImage *colorImg = 
    cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 3);

  // It is necessary to instantiate a stereoCam object each
  // time to get new frames.
  stereoCam cam;
  // Capture an image with colors and another with co-ordinates
  cam.doStereoFrame(colorImg, NULL, coordImg, NULL);

  // Get information from the co-ordinates image
  int coordImgHeight = coordImg->height;
  int coordImgWidth = coordImg->width;
  int coordImgChannels = coordImg->nChannels;
  // pointer access to raw data for fast element access
  double *coordImgData = (double *)coordImg->imageData;
  // Get the number of items in each row
  int coordImgRowCount = coordImg->widthStep/(sizeof(double));

  // Get information from the colors image
  int colorImgHeight = colorImg->height;
  int colorImgWidth = colorImg->width;
  int colorImgChannels = colorImg->nChannels;
  // pointer access to raw data for fast element access
  char *colorImgData = (char *)colorImg->imageData;
  // Get the number of items in each row
  int colorImgRowCount = colorImg->widthStep/(sizeof(char));

  double coordVal;
  int colorVal;
  int startIndex = 0;
  int endIndex = 25;

  ArNetPacket dataPacket;
  // Fill packet with header information
  dataPacket.byte4ToBuf(endIndex - startIndex);
  dataPacket.byte4ToBuf(endIndex - startIndex);
  dataPacket.byte4ToBuf(coordImgChannels);

  for (int i = startIndex; i < endIndex; i++) {
    for (int j = startIndex; j < endIndex; j++) {
      // fill coordinate information
      for (int k = 0; k < coordImgChannels; k++) {
	coordVal = 
	  coordImgData[i*coordImgRowCount + j*coordImgChannels + k]; 
	dataPacket.doubleToBuf(coordVal);
      }
      // pack color information
      for (int k = 0; k < colorImgChannels; k++) {
	colorVal = 
	  colorImgData[i*colorImgRowCount + j*colorImgChannels + k]; 
	dataPacket.byteToBuf(colorVal);
      }
    }
  }

  // send packet to client
  serverClient->sendPacketTcp(&dataPacket);
  */
}


// Adds stereo camera data service to the server
void SensorDataStereoCam::addData()
{
  myServer->addData("getSensorDataStereoCam",// packet type name
                    "send StereoCamera data",// short description
	  	    &(this->mySendFtr),	// callback functor
                    "no arguments",	// description of arguments
		 			// needed from client
		    "sends a packet containing stereocam readings");
}
