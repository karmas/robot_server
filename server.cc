/* This program creates a server which resides on a robot. The server
 * provides the following Aria services.
 *
   ArServerInfoRobot
   ArServerInfoSensor
   ArServerModeStop
   ArServerModeRatioDrive
   ArServerModeWander

 * An additional service for laser readings converted to 3d co-ordinates
 * is provided by PCLdata class.
 *
 * Multiple instances of the server program can be run on multiple robots.
 * For example:
 *   ./server -sp 7000
 * will start the server on port 7000
 *
 * To use with MobileSim run it with first:
 *   MobileSim -R p3dx
 *
 * Then each instance of server program will generate a new robot on
 * MobileSim.
 *
 * command line argument -cl
 * needed to connect laser
 *
 * command line argument -tilt n
 * specifies n as the tilt value in degrees for laser
 * e.g. -tilt 15 
 *      means tilted upwards 15 degrees from plane parallel to ground
 *
 * command line argument -maxrange n
 * readings further than this distance will be discarded
 * n is in millimeters
 *
 * command line argument -minrange n
 * readings lower than this distance will be discarded
 * n is in millimeters
 */

#include "Aria.h"
#include "ArNetworking.h"

#include <iostream>
#include <iomanip>
#include <list>
#include <map>
#include <ctime>
using namespace std;

// some message display routines
void echo(const string &msg)
{
  cout << "\t" << msg << endl;
}
void echo(const string &id, const int value)
{
  cout << "\t" << id << " = " << value << endl;
}
void echo(const string &id, const double value)
{
  cout << "\t" << id << " = " << value << endl;
}
void echo(const string &id, const string &value)
{
  cout << "\t" << id << " = " << value << endl;
}

// used to pack x,y,z values together
struct A3dpoint {
  A3dpoint(double v1, double v2, double v3)
    : x(v1), y(v2), z(v3) {}
  double x;
  double y;
  double z;
};

// distinguish invalid values
const int INVALID = -11;

// This class houses the method which is called when a request packet
// is of type "getPCL". 
class PCLdata {
public:
  PCLdata(ArServerBase *server, ArRobot *robot, int tilt,
          int maxRange, int minRange);
  void getData(ArServerClient *serverClient, ArNetPacket *packet);

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

const double PCLdata::toRadian = pi/180;


// sets the laser pointer now to the first laser in the robot
// also sets maxrange and minrange to laser max and zero if the
// values are invalid
PCLdata::PCLdata(ArServerBase *server, ArRobot *robot, int tilt,
                 int maxRange, int minRange)
  : myServer(server), myRobot(robot), myTilt(tilt),
    myMaxRange(maxRange), myMinRange(minRange),
    pclftr(this, &PCLdata::getData), myLaser(NULL)
{
  map<int, ArLaser *> *laserMap = myRobot->getLaserMap();
  map<int, ArLaser *>::iterator it;

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
}

/* @param serverClient: Connection manager between the server and the 
 * 	client. It is provided by the Aria framework and is used to
 * 	transmit a packet to client.
 * @param packet: It is received from client. It does not exist for
 * 	this request.
 * @func: Converts laser readings into 3d co-ordinates and stores them
 * 	in a packet which is sent to the client. The packet format is:
 * 	TIME STAMP (value returned by time function)
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
 */
void PCLdata::getData(ArServerClient *serverClient, ArNetPacket *packet)
{
  const std::list<ArSensorReading *> * readings = NULL;
  std::list<ArSensorReading *>::const_iterator it;
  const ArSensorReading *reading = NULL;

  double localX = 0.0;
  double localY = 0.0;
  double localZ = 0.0;
  double globalX = 0.0;
  double globalY = 0.0;
  // angle between local x-axis and global x-axis
  double alpha = 0.0;

  int readingRange = 0;
  vector<A3dpoint> points;

  ArNetPacket pclPacket;
  // Time stamp the packet
  pclPacket.byte4ToBuf(static_cast<int>(time(NULL)));
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
      // convert to local co-ordinates for sensor
      localX = reading->getLocalX() * cos(myTilt*toRadian);
      localY = reading->getLocalY();
      localZ = reading->getLocalX() * sin(myTilt*toRadian);

      // maybe add robot's height to z value
      localZ += 0.0;

      // find the x and y values in the global unrotated axis
      alpha = reading->getThTaken();
      globalX = localX*cos(alpha*toRadian) - localY*sin(alpha*toRadian);
      globalY = localY*cos(alpha*toRadian) + localX*sin(alpha*toRadian);

      // translate on x-y plane to convert to global co-ordinates
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


// laser testing routine
void testLaser(ArRobot *robot)
{
  ArLaser *laser = robot->findLaser(1);

  const std::list<ArSensorReading *> * readings = NULL;
  std::list<ArSensorReading *>::const_iterator it;
  ArSensorReading *reading = NULL;

  const int WIDTH = 8;
  const string SEP = "||";

  cout << "\t LASER READINGS" << endl;
  cout << setw(WIDTH) << "localX" << SEP;
  cout << setw(WIDTH) << "localY" << SEP;
  cout << setw(WIDTH) << "range" << SEP;
  cout << setw(WIDTH) << "sensorTh" << SEP;
  cout << setw(WIDTH) << "robotTh" << SEP;
  cout << endl;

  int i = 0;
  readings = laser->getRawReadings();
  for (it = readings->begin(); it != readings->end(); it++) {
    ++i;
    if (i % 10 == 0) {
      reading = (*it);
      // check if the laser reading is valid
      if (!reading->getIgnoreThisReading()) {
	cout << setw(WIDTH) << reading->getLocalX() << SEP;
	cout << setw(WIDTH) << reading->getLocalY() << SEP;
	cout << setw(WIDTH) << reading->getRange() << SEP;
	cout << setw(WIDTH) << reading->getSensorTh() << SEP;
	cout << setw(WIDTH) << reading->getThTaken() << SEP;
	cout << endl;
      }
    }
  }
  cout << endl;
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
  cout << "Here are the following options" << endl;
  cout << "\t" << tiltArg << " in degrees" <<  endl;
  cout << "\t" << maxRangeArg << " in mm" << endl;
  cout << "\t" << minRangeArg << " in mm" << endl;

  // try to find tilt index
  if (parser->checkArgument(tiltArg)) {
    index = getIndexForArg(argc, argv, tiltArg);
    if (index != -1) {
      *tilt = atoi(argv[index]);
      cout << "laser tilt = " << tilt << " degrees" << endl;
    }
  }

  // try to find max range
  if (parser->checkArgument(maxRangeArg)) {
    index = getIndexForArg(argc, argv, maxRangeArg);
    if (index != -1) {
      *maxRange = atoi(argv[index]);
      cout << "max range for laser is " << maxRange << " mm" << endl;
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
      cout << "min range for laser is " << maxRange << " mm" << endl;
    }
    else {
      echo("you did not give value for min range");
    }
  }
}


// main program
int main(int argc, char **argv)
{
  // necessary initialization of Aria framework
  Aria::init();

  // parse command line arguments
  ArArgumentParser parser(&argc, argv);
  parser.addDefaultArgument("-laserDegrees 180 -laserIncrement half");
  parser.loadDefaultArguments();

  int tilt = 0;
  int maxRange = INVALID;
  int minRange = INVALID;
  getCommandLineArguments(&parser, argc, argv, 
                          &tilt, &maxRange, &minRange);

  // connect to robot on which the server will exist
  ArRobot robot;
  ArRobotConnector robotConnector(&parser, &robot);
  if (!robotConnector.connectRobot()) {
    echo("unable to connect robot");
    Aria::exit(1);
  }

  // connect to the laser
  ArLaserConnector laserConnector(&parser, &robot, &robotConnector);
  // ArLaserConnector uses the command line arguments or configuration
  // file to configure the laser so parse those first
  Aria::parseArgs();
  if (!laserConnector.connectLasers()) {
    echo("unable to connect to laser");
    Aria::exit(1);
  }

  // add sonar to robot otherwise robot will not move forward
  ArSonarDevice sonar;
  robot.addRangeDevice(&sonar);

  // Create server and open port to the server
  // Default port is 7272
  ArServerBase server;
  ArServerSimpleOpener serverOpener(&parser);
  // Parse command line arguments which may contain user defined
  // server port number
  Aria::parseArgs();
  if (!serverOpener.open(&server)) {
    echo("unable to open server");
    Aria::exit(1);
  }

  // checking of command line arguments
  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed()) {
    echo("unable to parse command line arguments");
    Aria::exit(1);
  }

  // Information about robot such as speed and heading
  ArServerInfoRobot serverInfoRobot(&server, &robot);
  // Information from laser
  ArServerInfoSensor serverInfoSensor(&server, &robot);

  // Some modes which allow the robot to stop, drive or wander when
  ArServerModeStop modeStop(&server, &robot);
  ArServerModeRatioDrive modeRatioDrive(&server, &robot);
  ArServerModeWander modeWander(&server, &robot);

  // This object handles sending of PCL data packets from server 
  PCLdata pcl(&server, &robot, tilt, maxRange, minRange);

  // Need to add the new type of packet to the server since it is not
  // built into the Aria library
  server.addData("getPCL",		// packet type name
                 "send PCL data",	// short description
		 &pcl.pclftr,		// the functor which is called
		 			// when client requests this
					// type of packet
                 "no arguments",	// description of arguments
		 			// needed from client
		 "sends a packet containing 3d co-ordinates");

  /*
  ArServerHandlerCommands commands(&server);
  ArServerSimpleComUC uCCommands(&commands, &robot);
  */

  // Enable the motors
  robot.enableMotors();
  // Run the robot and the server
  robot.runAsync(true);
  server.runAsync();

  // Let the robot wait for manual exit from user
  robot.waitForRunExit();

  // Necessary closing of Aria framework
  Aria::shutdown();
}
