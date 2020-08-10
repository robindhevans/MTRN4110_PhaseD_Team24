// File:          e-puck_controller.cpp
// Date:          21/06/2020
// Description:   simple micro-mouse controller that follows path commands read in 
//                from a file. Three distance sensors added to the turrent slot
//                have a lookup table modelled after the Sharp GP2D120.
//                https://www.pololu.com/file/0J157/GP2D120-DATA-SHEET.pdf
// Author:        Robin Evans, UNSW

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Motor.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

//#define PATH_PLAN_FILE_NAME "C:/Users/u_da_/Documents/UNSW/Year3/Term2/MTRN/Assignment1/MTRN4110-Assignment-1/z1234567_MTRN4110_PhaseA/controllers/e-puck_controller/PathPlan.txt"
#define TRUE 1
#define FALSE 0
#define PI 3.14159
#define WALL_DETECT_DIST 2400
#define WHEEL_RADIUS 0.02
#define AXEL_LENGTH 0.0566

#define MAX_TRANS_VELOCITY 0.25
//in m/s
#define MAX_ROT_VELOCITY 6.28
#define STEERING_VELOCITY_CONST 0.4
//in rad/s

using namespace webots;
using namespace std;

//enum Heading {North, South, East, West};

//Function to rotate the robot left
void rotateCCW(Robot* robot, PositionSensor* right_ps, PositionSensor* left_ps, Motor* right_motor, Motor* left_motor , int timeStep) {
  
  // get current position
  robot->step(timeStep);
  double r_start_pose = right_ps->getValue();
  //cout << r_start_pose << endl;
  double l_start_pose = left_ps->getValue();
  
  // set target position
  double wheel_rot_targ = (AXEL_LENGTH * PI / 2) / WHEEL_RADIUS;
  //cout << wheel_rot_targ << endl;
  double r_wheel_targ = r_start_pose + wheel_rot_targ / 2;
  //cout << r_wheel_targ << endl;
  double l_wheel_targ = l_start_pose - wheel_rot_targ / 2;
  double deltaTheta = abs(r_wheel_targ - r_start_pose);
  
  //load target position
  right_motor->setPosition(r_wheel_targ);
  left_motor->setPosition(l_wheel_targ);
  right_motor->setVelocity(STEERING_VELOCITY_CONST * MAX_ROT_VELOCITY);
  left_motor->setVelocity(STEERING_VELOCITY_CONST * MAX_ROT_VELOCITY);
  
  //wait until complete
  while(robot->step(timeStep) != -1 && deltaTheta > 0.001) {
    deltaTheta = abs(r_wheel_targ - right_ps->getValue());
    //cout << deltaTheta << endl;
    //cout << right_ps->getValue() << endl;
    //cout << right_motor->getVelocity() << endl;
  }
};

//Function to rotate the robot right
void rotateCW(Robot* robot, PositionSensor* right_ps, PositionSensor* left_ps, Motor* right_motor, Motor* left_motor , int timeStep) {
  
  // get current position
  robot->step(timeStep);
  double r_start_pose = right_ps->getValue();
  //cout << r_start_pose << endl;
  double l_start_pose = left_ps->getValue();
  
  // set target position
  double wheel_rot_targ = (AXEL_LENGTH * PI / 2) / WHEEL_RADIUS;
  //cout << wheel_rot_targ << endl;
  double r_wheel_targ = r_start_pose - wheel_rot_targ / 2;
  //cout << r_wheel_targ << endl;
  double l_wheel_targ = l_start_pose + wheel_rot_targ / 2;
  double deltaTheta = abs(r_wheel_targ - r_start_pose);
  
  //load target position
  right_motor->setPosition(r_wheel_targ);
  left_motor->setPosition(l_wheel_targ);
  right_motor->setVelocity(STEERING_VELOCITY_CONST * MAX_ROT_VELOCITY);
  left_motor->setVelocity(STEERING_VELOCITY_CONST * MAX_ROT_VELOCITY);
  
  //wait until complete
  while(robot->step(timeStep) != -1 && deltaTheta > 0.001) {
    deltaTheta = abs(r_wheel_targ - right_ps->getValue());
    //cout << deltaTheta << endl;
    //cout << right_ps->getValue() << endl;
    //cout << right_motor->getVelocity() << endl;
  }
};

//Function to move the robot forward one cell
void forward(Robot* robot, PositionSensor* right_ps, PositionSensor* left_ps, Motor* right_motor, Motor* left_motor , int timeStep) {
  
  // get current position
  robot->step(timeStep);
  double r_start_pose = right_ps->getValue();
  //cout << r_start_pose << endl;
  double l_start_pose = left_ps->getValue();
  
  // set target position
  double wheel_trans_targ = (0.33 / WHEEL_RADIUS) / 2;
  double r_wheel_targ = r_start_pose + wheel_trans_targ;
  double l_wheel_targ = l_start_pose + wheel_trans_targ;
  //cout << wheel_trans << endl;
  double deltaTheta = abs(wheel_trans_targ - r_start_pose);
  //load target position
  right_motor->setPosition(r_wheel_targ);
  left_motor->setPosition(l_wheel_targ);
  right_motor->setVelocity(MAX_ROT_VELOCITY);
  left_motor->setVelocity(MAX_ROT_VELOCITY);
  
  //wait until complete
  while(robot->step(timeStep) != -1 && deltaTheta > 0.002) {
    deltaTheta = abs(r_wheel_targ - right_ps->getValue());
    //cout << deltaTheta << endl;
    //cout << right_ps->getValue() << endl;
    //cout << right_motor->getVelocity() << endl;
  }
};

//Function to update cell position based on current heading
void update_cell(Heading my_heading, int& row, int& col) {
  if(my_heading == North) row--;
  if(my_heading == South) row++;
  if(my_heading == East) col++;
  if(my_heading == West) col--;
        
  // if south, F, heading doesnt change, row increases
  // if north, F, heading doesnt change, row decreases
  // if east, F, heading doesnt change, col increases
  // if west, F, heading doesnt change, col decreases
  //cout <<  "Row = " << row;
  //cout <<  " Col = " << col << endl;  
};

//Function to updated heading based on command input
void update_heading(Heading& my_heading,char c) {
  switch(my_heading) {
    case(North):
      if(c == 'R') my_heading = East;
      if(c == 'L') my_heading = West;
      break;
    case(South):
      if(c == 'R') my_heading = West;
      if(c == 'L') my_heading = East;
      break;
    case(East):
      if(c == 'R') my_heading = South;
      if(c == 'L') my_heading = North;
      break;
    case(West):
      if(c == 'R') my_heading = North;
      if(c == 'L') my_heading = South;
  }
  //cout << my_heading << endl;
};

//Function to detect walls
bool wall_detect(Robot* robot, DistanceSensor* ds, int timeStep) {
  robot->step(timeStep);
  double val = ds->getValue();
  bool wall = FALSE;
  val > WALL_DETECT_DIST ? wall = TRUE : wall = FALSE;
  //cout << val << endl;
  return wall;
};
// front wall detection could use two front-facing distance sensors.
// to correct any error in dead-reckoning using motor encoders later.
  
  
// Main control logic
int z5197018_MTRN4110_PhaseA() {
  // create the Robot instance.
  Robot *robot = new Robot();
  int timeStep = (int)robot->getBasicTimeStep();
  //Keyboard Read
  Keyboard keyboard;
  keyboard.enable(10);
  char key = 0;
  while(robot->step(timeStep) != -1 && key != 'Q') {
    key = keyboard.getKey();
    if(key == 'W') cout << "UP" << endl;
    
  }
  int row = 0;
  int col = 0;
  int step_counter = 0;
  char c;
  //initialise variables
  
  //double bot_pose [3] = {0};
  //bot_pose[0] = col*0.165-0.66; // x coord (WRT center)
  //bot_pose[1] = row*0.165-0.33; // y coord
  Heading my_heading;
  //bot pose x y theta, where theta is heading measured CCW+ from pos x axis
  bool wall_right = FALSE;
  bool wall_left = FALSE;
  bool wall_front = FALSE;
  //start input stream
  cout << "Start - Read path plan from " << PATH_PLAN_FILE_NAME << ':' << endl;
  
  ifstream path_plan;
  //open and read path plan text file
  path_plan.open(PATH_PLAN_FILE_NAME);
  if (path_plan.is_open()) {
    while(path_plan.get(c)) {
      cout << c;
    }
    cout << endl;
    cout << "Done - Path plan read!" << endl;
  }
  else {
    cerr << "Error: Path Plan file could not be openned" << endl;
    path_plan.close();
    return 1;
  }
  cout << "Start - Execute start plan!" << endl;
  
  path_plan.clear();
  path_plan.seekg(0, path_plan.beg);
  cout << "Step: 00,";
  //initialise row, col and heading based on file instructions
  row = path_plan.get()-48;
  cout << " Row: " << row << ',';
  col = path_plan.get()-48;
  cout << " Column: " << col << ',';
  //cout << "Initial Row = " << row << endl;
  //cout << "Initial Col = " << col << endl;
  c = path_plan.get();
  switch(c) {
    case('N'): my_heading = North;
      cout << " Heading: N,";
      //bot_pose[2] = PI/2;
      break;
    case('S'): my_heading = South;
      cout << " Heading: S,";    
      //bot_pose[2] = 3*PI/2;
      break;
    case('E'): my_heading = East;
      cout << " Heading: E,";
      break;
    case('W'): my_heading = West;
      cout << " Heading: W,";    
      //bot_pose[2] = PI;
      break;
    default: cerr << "Error: Invalid initial heading" << endl;
    return 1;
  }
  //cout << "Initial Pose = (" << bot_pose[0] << " , " << bot_pose[1] << " , " << bot_pose[2] << endl;
  
  // initialise devices
  DistanceSensor *left_ds = robot->getDistanceSensor("ds2");
  left_ds->enable(timeStep);
  DistanceSensor *right_ds = robot->getDistanceSensor("ds1");
  right_ds->enable(timeStep);
  DistanceSensor *front_ds = robot->getDistanceSensor("ds0");
  front_ds->enable(timeStep);
  PositionSensor *right_ps = robot->getPositionSensor("right wheel sensor");
  right_ps->enable(timeStep);
  PositionSensor *left_ps = robot->getPositionSensor("left wheel sensor");
  left_ps->enable(timeStep);
  Motor *right_motor = robot->getMotor("right wheel motor");
  Motor *left_motor = robot->getMotor("left wheel motor");

  //determine initial surroundings:
  wall_left = wall_detect(robot, left_ds, timeStep);
  cout << " Left Wall: ";
  wall_left == FALSE ? cout << "N," : cout << "Y,";
  //cout << wall_left;
  wall_front = wall_detect(robot, front_ds, timeStep);
  cout << " Front Wall: ";
  wall_front == FALSE ? cout << "N," : cout << "Y,";
  //cout << wall_front;
  wall_right = wall_detect(robot, right_ds, timeStep);
  cout << " Right Wall: ";
  wall_right == FALSE ? cout << "N" << endl : cout << "Y" << endl;
  //cout << wall_right;
  
  //Read and follow commands from file
  while(path_plan.get(c)) {
    //cout << c << endl;
    switch(c) {
      case('F'):
        //cout << "driving forward one space" << endl;
        forward(robot, right_ps, left_ps, right_motor, left_motor , timeStep);
        update_cell(my_heading, row, col);
        wall_right = wall_detect(robot, right_ds, timeStep);
        //cout << wall_right;
        wall_left = wall_detect(robot, left_ds, timeStep);
        //cout << wall_left;
        wall_front = wall_detect(robot, front_ds, timeStep);
        //cout << wall_front << endl;
        break;
      case('R'):
        //cout << "rotating right 90 deg" << endl;
        rotateCW(robot, right_ps, left_ps, right_motor, left_motor , timeStep);
        update_heading(my_heading, c);
        wall_right = wall_detect(robot, right_ds, timeStep);
        //cout << wall_right;
        wall_left = wall_detect(robot, left_ds, timeStep);
        //cout << wall_left;
        wall_front = wall_detect(robot, front_ds, timeStep);
        //cout << wall_front << endl;
        break;
      case('L'):
        //cout << "rotating left 90 deg" << endl;
        rotateCCW(robot, right_ps, left_ps, right_motor, left_motor , timeStep);
        update_heading(my_heading, c);
        wall_right = wall_detect(robot, right_ds, timeStep);
        //cout << wall_right;
        wall_left = wall_detect(robot, left_ds, timeStep);
        //cout << wall_left;
        wall_front = wall_detect(robot, front_ds, timeStep);
        //cout << wall_front << endl;
        break;
      default:
        cerr << "Invalid command encountered. Continuing..." << endl;
    }
    step_counter++;
    step_counter <= 9 ? cout << "Step: 0" << step_counter << ", " 
    : cout << "Step: " << step_counter << ", ";
    cout << "Row: " << row << ", " << "Column: " << col << ", ";
    switch(my_heading) {
      case(North):
        cout << "Heading: N, ";
        break;
      case(South):
        cout << "Heading: S, ";
        break;
      case(East):
        cout << "Heading: E, ";
        break;
      case(West):
        cout << "Heading: W, ";
    }
    wall_left == TRUE ? cout << "Left Wall: Y, " : cout << "Left Wall: N, ";
    wall_front == TRUE ? cout << "Front Wall: Y, " : cout << "Front Wall: N, ";
    wall_right == TRUE ? cout << "Right Wall: Y" : cout << "Right Wall: N";
    cout << endl;
  }
    cout << "Done - Path plan executed!" << endl;

  if (path_plan.is_open()) 
    path_plan.close();

  delete robot;
  return 0;
}