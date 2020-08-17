// File:          TheA-maze-ingRacers_z5204873_z5197018.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <vector>
#include <iostream>
#include <webots/Robot.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

enum heading {North = 0, East, South, West};
struct walls_detected {
  bool N = false;
  bool E = true;
  bool S = false;
  bool W = true;
  } robot_walls, map_walls;
  //struct robot_walls holds last reading from sensors, directions are from robot
  //reference frame
  //struct map_walls holds last wall directions in map reference frame.

void translate_NESW(heading my_heading, walls_detected * robot_walls, walls_detected * map_walls) {
  //translates wall locations from robot reference frame to map reference frame
  //use map_walls when determining location
  switch (my_heading) {
    case(North):
      map_walls->N = robot_walls->N;
      map_walls->E = robot_walls->E;
      map_walls->S = robot_walls->S;
      map_walls->W = robot_walls->W;
      break;
    case(East):
      map_walls->N = robot_walls->W;
      map_walls->E = robot_walls->N;
      map_walls->S = robot_walls->E;
      map_walls->W = robot_walls->S;
      break;
    case(South):
      map_walls->N = robot_walls->S;
      map_walls->E = robot_walls->W;
      map_walls->S = robot_walls->N;
      map_walls->W = robot_walls->E;
      break;
    case(West):
      map_walls->N = robot_walls->E;
      map_walls->E = robot_walls->S;
      map_walls->S = robot_walls->W;
      map_walls->W = robot_walls->N;
  }
}

int main(int argc, char **argv) {
  vector<vector<int>> paths;
  heading my_heading = East;
  translate_NESW(my_heading, &robot_walls, &map_walls);
  //cout << map_walls.N << map_walls.E << map_walls.S << map_walls.W << endl;
  // create the Robot instance.
  Robot *robot = new Robot();
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
