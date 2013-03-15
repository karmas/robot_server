/* This program creates a server which resides on a robot. The server
 * provides the following Aria services.
 *
   ArServerModeStop
   ArServerModeRatioDrive
   ArServerModeWander

 * An additional service for laser readings converted to 3d co-ordinates
 * is provided by PCLdata class.
 *
 * Start a server at TCP port 7272 on a robot at TCP port 8101.
 *   server -rrtp 8101 -sp 7272
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
 *
 * It is easier to put the command line arguments in a single line in a
 * file and start the program by specifying the file with -file arg.
 * e.g. server -file config.txt
 */


#include "utils.h"
#include "SensorData.h"


// main program
int main(int argc, char **argv)
{
  // necessary initialization of Aria framework
  Aria::init();

  ArArgumentParser parser(&argc, argv);

  // configuration variables
  int tilt = 0;
  A3dpoint laserToRobotTranslation(0,0,0);
  int maxRange = INVALID;
  int minRange = INVALID;

  // check for config file
  const char *fileName = getConfigFile(parser);
  if (fileName != NULL) {
    parser.addDefaultArgumentFile(fileName);
    // load arguments from file
    parser.loadDefaultArguments();
    // get configuration information from arguments file
    configureRobot(parser, &tilt, laserToRobotTranslation,
		   &maxRange, &minRange);
  }

  // parse command line arguments for configuration information
  Aria::parseArgs();

  // connect to robot on which the server will exist
  ArRobot robot;
  ArRobotConnector robotConnector(&parser, &robot);
  if (!robotConnector.connectRobot()) {
    echo("unable to connect to robot");
    Aria::exit(1);
  }

  // connect to the laser
  ArLaserConnector laserConnector(&parser, &robot, &robotConnector);
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
  if (!serverOpener.open(&server)) {
    echo("unable to open server");
    Aria::exit(1);
  }

  // warn about wrong command line arguments
  parser.checkHelpAndWarnUnparsed();

  // Some modes which allow the robot to stop, drive or wander when
  ArServerModeStop modeStop(&server, &robot);
  ArServerModeRatioDrive modeRatioDrive(&server, &robot);
  ArServerModeWander modeWander(&server, &robot);

  // This object handles sending of laser data packets from server 
  SensorDataLaser sdl(&server, &robot, tilt, laserToRobotTranslation,
      		      maxRange, minRange);

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
