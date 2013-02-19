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
 */


#include "utils.h"
#include "PCLdata.h"


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
