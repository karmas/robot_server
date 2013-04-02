#include <map>
#include <ctime>
#include <sys/time.h>
#include <cassert>

#include "stereoCamera.h"
#include "SensorData.h"

#define STEREO_CAM_SUBSAMPLE


const double SensorData::pi = 3.14159165f;
const double SensorData::toRadian = pi/180;

SensorData::SensorData(ArServerBase *server, ArRobot *robot)
  : myServer(server), myRobot(robot)
{
}

// fill packet with robot location and heading
void SensorData::robotToBuf(ArNetPacket *packet)
{
  assert(packet);
  packet->doubleToBuf(myRobot->getX());
  packet->doubleToBuf(myRobot->getY());
  packet->doubleToBuf(myRobot->getTh());
}



///////////////////////////////
//  SensorDataLaser
///////////////////////////////


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

  static ArNetPacket pclPacket;
  robotToBuf(&pclPacket);

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

  pclPacket.finalizePacket();
  serverClient->sendPacketTcp(&pclPacket);
  pclPacket.empty();
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


///////////////////////////////
//  SensorDataStereoCam
///////////////////////////////

// Set various properties for image capture
SensorDataStereoCam::SensorDataStereoCam(ArServerBase *server, 
    ArRobot *robot)
  : SensorData(server, robot),
    mySendFtr(this, &SensorDataStereoCam::send),
    mySendFtr2(this, &SensorDataStereoCam::send2),
    myPTU(new ArDPPTU(myRobot)),
    myCam(new stereoCam),
    myCaptureWidth(320),
    myCaptureHeight(240),
    myPointSize(3*sizeof(COORDINATE_TYPE) + 3*sizeof(char)),
    myRowIncrement(1),
    myColIncrement(1)
{
#ifdef STEREO_CAM_SUBSAMPLE
  myRowIncrement = 2;
  myColIncrement = 2;
#endif
  addData();
}

// properly shut down the stereo camera
SensorDataStereoCam::~SensorDataStereoCam()
{
  delete myPTU;
  delete myCam;
}

// Gets global readings from stereo camera which are packed into short 
// datatypes. Hence the range for data is about [-32m, 32m]  from the 
// starting location. This function only sends upper half of the image
//
// Format of data packet
// -----------------
// ROBOT X CO-ORDINATE (DOUBLE)
// ROBOT Y CO-ORDINATE (DOUBLE)
// ROBOT HEADING (DOUBLE measured in degrees)
// NUMBER OF PIXELS
// PIXEL 1 X (SHORT)
// PIXEL 1 Y (SHORT)
// PIXEL 1 Z (SHORT)
// PIXEL 1 COLOR R (CHAR)
// PIXEL 1 COLOR G (CHAR)
// PIXEL 1 COLOR B (CHAR)
// PIXEL 2 X
// PIXEL 2 Y
// PIXEL 2 Z
// PIXEL 2 COLOR R
// PIXEL 2 COLOR G
// PIXEL 2 COLOR B
//   .
//   .
// -----------------
void SensorDataStereoCam::send(ArServerClient *serverClient, 
    ArNetPacket *packet)
{
  IplImage *coordImg = 
    cvCreateImage(cvSize(myCaptureWidth,myCaptureHeight), IPL_DEPTH_64F, 3);
  IplImage *colorImg = 
    cvCreateImage(cvSize(myCaptureWidth,myCaptureHeight), IPL_DEPTH_8U, 3);

  // figure out max points to fill data packet
  static ArNetPacket dataPacket;
  static const int MAX_POINTS =
    (dataPacket.MAX_DATA_LENGTH - 
     (3*sizeof(double) + sizeof(int))) / myPointSize;

  // Fill robot location and heading
  robotToBuf(&dataPacket);

  // Capture an image with colors and another with co-ordinates
  myCam->doStereoFrame(colorImg, NULL, coordImg, NULL,
      myPTU->getPan(), myPTU->getTilt(), 
      myRobot->getX(), myRobot->getY(), myRobot->getTh());

  // Get information from the co-ordinates image
  int coordImgHeight = coordImg->height;
  int coordImgWidth = coordImg->width;
  int coordImgChannels = coordImg->nChannels;
  // pointer access to raw data for fast element access
  double *coordImgData = (double *)coordImg->imageData;
  // Get the number of items in each row
  int coordImgRowCount = coordImg->widthStep/(sizeof(double));

  // Get information from the colors image
  int colorImgChannels = colorImg->nChannels;
  // pointer access to raw data for fast element access
  char *colorImgData = (char *)colorImg->imageData;
  // Get the number of items in each row
  int colorImgRowCount = colorImg->widthStep/(sizeof(char));

  // subsampling variables
  static int rowStart = 0;
  static int rowEnd = coordImgHeight/2;

  // for current packet start here
  static int currRowStart = rowStart;
  // reset if we have sent the last row of the image
  if (currRowStart >= rowEnd) currRowStart = rowStart;

  double origX, origY, origZ;
  int coordIndex;
  int colorIndex;
  int addedPoints = 0;
  // storage for valid data
  std::vector<short> points;
  std::vector<char> colors;

  // go through and select only valid points from data
  int i;
  bool loopExit = false;
  for (i = currRowStart; i < rowEnd; i += myRowIncrement) {
    if (loopExit) break;

    for (int j = 0; j < coordImgWidth; j += myColIncrement) {
      // get indices to co-ordinate and color data
      coordIndex = i*coordImgRowCount + j*coordImgChannels;
      colorIndex = i*colorImgRowCount + j*colorImgChannels;

      // directly access co-ordinate information
      origX = coordImgData[coordIndex]; 
      origY = coordImgData[coordIndex + 1]; 
      origZ = coordImgData[coordIndex + 2]; 

      // skip if invalid
      if (invalidPoint(origX, origY, origZ)) continue;

      // store coordinates and color
      points.push_back(origX);
      points.push_back(origY);
      points.push_back(origZ);
      colors.push_back(colorImgData[colorIndex + 2]);
      colors.push_back(colorImgData[colorIndex + 1]);
      colors.push_back(colorImgData[colorIndex]);
      addedPoints++;

      // exit loops if we reach max number of points
      if (addedPoints >= MAX_POINTS) {
	loopExit = true;
	break;
      }
    }
  }
  // next time start here
  currRowStart = i;

  // Fill packet with header information
  dataPacket.byte4ToBuf(points.size() / 3);
  // fill packet with point co-ordinate and color
  for (size_t i = 0; i < points.size(); i += 3) {
    dataPacket.byte2ToBuf(points[i]);
    dataPacket.byte2ToBuf(points[i+1]);
    dataPacket.byte2ToBuf(points[i+2]);
    dataPacket.byteToBuf(colors[i]);
    dataPacket.byteToBuf(colors[i+1]);
    dataPacket.byteToBuf(colors[i+2]);
  }

  dataPacket.finalizePacket();
  serverClient->sendPacketTcp(&dataPacket);
  dataPacket.empty();

  cvReleaseImage(&colorImg);
  cvReleaseImage(&coordImg);
}

// send the lower half of image
void SensorDataStereoCam::send2(ArServerClient *serverClient, 
    ArNetPacket *packet)
{
  IplImage *coordImg = 
    cvCreateImage(cvSize(myCaptureWidth,myCaptureHeight), IPL_DEPTH_64F, 3);
  IplImage *colorImg = 
    cvCreateImage(cvSize(myCaptureWidth,myCaptureHeight), IPL_DEPTH_8U, 3);

  // figure out max points to fill data packet
  static ArNetPacket dataPacket;
  static const int MAX_POINTS =
    (dataPacket.MAX_DATA_LENGTH - 
     (3*sizeof(double) + sizeof(int))) / myPointSize;

  // Fill robot location and heading
  robotToBuf(&dataPacket);

  // Capture an image with colors and another with co-ordinates
  myCam->doStereoFrame(colorImg, NULL, coordImg, NULL,
      myPTU->getPan(), myPTU->getTilt(), 
      myRobot->getX(), myRobot->getY(), myRobot->getTh());

  // Get information from the co-ordinates image
  int coordImgHeight = coordImg->height;
  int coordImgWidth = coordImg->width;
  int coordImgChannels = coordImg->nChannels;
  // pointer access to raw data for fast element access
  double *coordImgData = (double *)coordImg->imageData;
  // Get the number of items in each row
  int coordImgRowCount = coordImg->widthStep/(sizeof(double));

  // Get information from the colors image
  int colorImgChannels = colorImg->nChannels;
  // pointer access to raw data for fast element access
  char *colorImgData = (char *)colorImg->imageData;
  // Get the number of items in each row
  int colorImgRowCount = colorImg->widthStep/(sizeof(char));

  // subsampling variables
  static int rowStart = coordImgHeight/2;
  static int rowEnd = coordImgHeight;

  // for current packet start here
  static int currRowStart = rowStart;
  // reset if we have sent the last row of the image
  if (currRowStart >= rowEnd) currRowStart = rowStart;

  double origX, origY, origZ;
  int coordIndex;
  int colorIndex;
  int addedPoints = 0;
  // storage for valid data
  std::vector<short> points;
  std::vector<char> colors;

  // go through and select only valid points from data
  int i;
  bool loopExit = false;
  for (i = currRowStart; i < rowEnd; i += myRowIncrement) {
    if (loopExit) break;

    for (int j = 0; j < coordImgWidth; j += myColIncrement) {
      // get indices to co-ordinate and color data
      coordIndex = i*coordImgRowCount + j*coordImgChannels;
      colorIndex = i*colorImgRowCount + j*colorImgChannels;

      // directly access co-ordinate information
      origX = coordImgData[coordIndex]; 
      origY = coordImgData[coordIndex + 1]; 
      origZ = coordImgData[coordIndex + 2]; 

      // skip if invalid
      if (invalidPoint(origX, origY, origZ)) continue;

      // store coordinates and color
      points.push_back(origX);
      points.push_back(origY);
      points.push_back(origZ);
      colors.push_back(colorImgData[colorIndex + 2]);
      colors.push_back(colorImgData[colorIndex + 1]);
      colors.push_back(colorImgData[colorIndex]);
      addedPoints++;

      // exit loops if we reach max number of points
      if (addedPoints >= MAX_POINTS) {
	loopExit = true;
	break;
      }
    }
  }
  // next time start here
  currRowStart = i;

  // Fill packet with header information
  dataPacket.byte4ToBuf(points.size() / 3);
  // fill packet with point co-ordinate and color
  for (size_t i = 0; i < points.size(); i += 3) {
    dataPacket.byte2ToBuf(points[i]);
    dataPacket.byte2ToBuf(points[i+1]);
    dataPacket.byte2ToBuf(points[i+2]);
    dataPacket.byteToBuf(colors[i]);
    dataPacket.byteToBuf(colors[i+1]);
    dataPacket.byteToBuf(colors[i+2]);
  }

  dataPacket.finalizePacket();
  serverClient->sendPacketTcp(&dataPacket);
  dataPacket.empty();

  cvReleaseImage(&colorImg);
  cvReleaseImage(&coordImg);
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
  myServer->addData("getSensorDataStereoCam2",// packet type name
                    "send StereoCamera data",// short description
	  	    &(this->mySendFtr2),	// callback functor
                    "no arguments",	// description of arguments
		 			// needed from client
		    "sends a packet containing stereocam readings");
}

// The stereo camera my assign points which do not exist so rule them out
bool SensorDataStereoCam::invalidPoint(double x, double y, double z)
{
  // reject closer than 2cm
  if (abs(x) < 20.0) return true;
  // reject below 1/2m
  if (z < -500.0) return true;
  return false;
}
