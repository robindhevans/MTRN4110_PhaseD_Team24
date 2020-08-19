//z5195019 PhaseA Epuck Controller

#include <webots/Robot.hpp>
// Added a new include file
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <fstream>
#include <string.h>



#define TIME_STEP 64
#define MAX_SPEED 6.28
#define ROTATE_STEP 2.2227
//90 deg arclength dvided by radius of wheels
#define TILE_STEP 8.25
//tile width divided by radius
#define TOLERANCE 0.005
#define PATH_PLAN_FILE_NAME "../../PathPlan.txt"



// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

typedef struct robdata robdata;
struct robdata {
  char heading;
    //North=0 - East=1 - South=2 - West=3
  int row;
  int col;
  int step;
  };
  
typedef struct robotsensors robotsensors;
struct robotsensors {
  DistanceSensor *front_ds;
  DistanceSensor *right_ds;
  DistanceSensor *left_ds;
  Motor *leftMotor;
  Motor *rightMotor;
  PositionSensor *left_en;
  PositionSensor *right_en;
  double l_target = 0.0;
  double r_target = 0.0;
  
  double l_pos = 0.0;
  double r_pos = 0.0;
  
  }; 

void checkwalls (robotsensors sensors, robdata robo);
void forward(int seq, Robot *robot, robotsensors &sensors, robdata &robo);
void left(Robot *robot, robotsensors &sensors, robdata &robo);
void right(Robot *robot, robotsensors &sensors, robdata &robo);

int main(int argc, char **argv) {
  //variables
  char pathPlan[50];
  robotsensors sensors;
  robdata robo;
  
  
  //initialise robot
  Robot *robot = new Robot();
  // get a handler to the motors.
  sensors.leftMotor = robot->getMotor("left wheel motor");
  sensors.rightMotor = robot->getMotor("right wheel motor");

  sensors.front_ds = robot->getDistanceSensor("F_DS");
  sensors.right_ds = robot->getDistanceSensor("R_DS");
  sensors.left_ds = robot->getDistanceSensor("L_DS");
  
  sensors.left_en = robot->getPositionSensor("left wheel sensor");
  sensors.right_en = robot->getPositionSensor("right wheel sensor");
  
  sensors.front_ds->enable(TIME_STEP);
  sensors.right_ds->enable(TIME_STEP);
  sensors.left_ds->enable(TIME_STEP);
  sensors.left_en->enable(TIME_STEP);
  sensors.right_en->enable(TIME_STEP);

  robot->step(TIME_STEP);


  sensors.leftMotor->setVelocity(0.8 * MAX_SPEED);
  sensors.rightMotor->setVelocity(0.8 * MAX_SPEED);

  
  // read file in path 
  ifstream pathFile;
  pathFile.open(PATH_PLAN_FILE_NAME, ios::in);
  if (!pathFile.is_open()) {
    cout << "Failed to open PathPlan" << endl;
    cout << "Exiting Controller..." << endl;
    exit(EXIT_FAILURE);
  }
  pathFile >> pathPlan;
  cout << pathPlan << endl;
  cout << "Done - Path plan read!" << endl;
  cout << "Start - Execute path plan!" << endl;
  
  
  robo.row = (int)pathPlan[0]-48;
  robo.col = (int)pathPlan[1]-48;
  robo.heading = pathPlan[2];
  robo.step = 2;
  
  int planLength = strlen(pathPlan);

  while (robo.step < planLength && robot->step(TIME_STEP) != -1){
    int seq = 0;
    //Start forward instruction
    if (pathPlan[robo.step] == 'F'){
      for(seq = 0; pathPlan[robo.step+seq] == 'F'; seq++){
      }
      forward(seq, robot, sensors, robo);
    }  
        
    //End forward instruction
    
    //start left rotation instruction
    else if (pathPlan[robo.step] == 'L'){
      left(robot, sensors, robo);
    }
    
    //start right rotate instruction
    else if (pathPlan[robo.step] == 'R'){
      right(robot, sensors, robo);
    }
    
    //print step information
    checkwalls(sensors, robo);
    robo.step++; 
  }

  cout << "Done - Path plan Executed" << endl;

  delete robot;
  pathFile.close();

  return 0;

}

void forward(int seq, Robot *robot, robotsensors &sensors, robdata &robo){
  sensors.leftMotor->setVelocity(0.8 * MAX_SPEED);
  sensors.rightMotor->setVelocity(0.8 * MAX_SPEED);
        
  sensors.leftMotor->setPosition(sensors.l_target + (seq*TILE_STEP));
  sensors.rightMotor->setPosition(sensors.r_target + (seq*TILE_STEP));
      
  sensors.l_target += seq*TILE_STEP;
  sensors.r_target += seq*TILE_STEP;
     
  int count = seq-1; 
  // check sensors and compare target position
  while (sensors.l_target-TOLERANCE >= sensors.l_pos && sensors.r_target-TOLERANCE >= sensors.r_pos){
  robot->step(TIME_STEP);
  sensors.l_pos = sensors.left_en->getValue();
  sensors.r_pos = sensors.right_en->getValue();
  
  //detect complete tiles and update console and position
    if(sensors.l_pos > sensors.l_target-(TILE_STEP*count+2*TOLERANCE)){
      switch (robo.heading){
        case 'S':
          robo.row += 1;
          break;
        case 'N':
          robo.row -= 1;
          break;
        case 'E':
          robo.col += 1;
          break;
        case 'W':
          robo.col -= 1;
          break;
      }
      if(count > 0){
        checkwalls(sensors, robo);
        robo.step++;
      }
      count--;
    }
  }
}

void left(Robot *robot, robotsensors &sensors, robdata &robo){
  sensors.leftMotor->setVelocity(0.4 * MAX_SPEED);
  sensors.rightMotor->setVelocity(0.4 * MAX_SPEED);

  sensors.leftMotor->setPosition(sensors.l_target - ROTATE_STEP);
  sensors.rightMotor->setPosition(sensors.r_target + ROTATE_STEP);
    
  sensors.l_target -= ROTATE_STEP;
  sensors.r_target += ROTATE_STEP;
  //read sesors to check position and robo  
  while (sensors.l_target+TOLERANCE <= sensors.l_pos && sensors.r_target-TOLERANCE >= sensors.r_pos){
    robot->step(TIME_STEP);
    sensors.l_pos = sensors.left_en->getValue();
    sensors.r_pos = sensors.right_en->getValue();   
  }
  //adjust robo.heading
  switch (robo.heading){
    case 'S':
      robo.heading = 'E';
      break;
    case 'N':
      robo.heading = 'W';
      break;
    case 'E':
      robo.heading = 'N';
      break;
    case 'W':
      robo.heading = 'S';
      break;
  }
}

void right(Robot *robot, robotsensors &sensors, robdata &robo){
  sensors.leftMotor->setVelocity(0.4 * MAX_SPEED);
  sensors.rightMotor->setVelocity(0.4 * MAX_SPEED);
  
  sensors.leftMotor->setPosition(sensors.l_target + ROTATE_STEP);
  sensors.rightMotor->setPosition(sensors.r_target - ROTATE_STEP);
    
  sensors.l_target += ROTATE_STEP;
  sensors.r_target -= ROTATE_STEP;
  //read sesors to check position and robo  
  while (sensors.l_target-TOLERANCE >= sensors.l_pos && sensors.r_target+TOLERANCE <= sensors.r_pos){
    robot->step(TIME_STEP);
    sensors.l_pos = sensors.left_en->getValue();
    sensors.r_pos = sensors.right_en->getValue();
  }
  //adjust robo.heading
  switch (robo.heading){
    case 'S':
      robo.heading = 'W';
      break;
    case 'N':
      robo.heading = 'E';
      break;
    case 'E':
      robo.heading = 'S';
      break;
    case 'W':
      robo.heading = 'N';
      break;
  }
}
  
  
void checkwalls (robotsensors sensors, robdata robo){
  //correct wall variables
  char Lwall;
  char Rwall;
  char Fwall;  
  
  if(sensors.left_ds->getValue() > 0.85){
    Lwall = 'Y';
  }else{
    Lwall = 'N';
  }
  
  if(sensors.front_ds->getValue() > 0.85){
    Fwall = 'Y';
  }else{ 
    Fwall = 'N';
  }
  
  if(sensors.right_ds->getValue() > 0.85){
    Rwall = 'Y';
  }else{
    Rwall = 'N';
  }
  
  printf("Step: %02d, ", robo.step-2);
  cout<<"Row: "<<robo.row<<", Column: "<<robo.col<<", Heading: "<<robo.heading;
  cout<<", Left Wall: "<<Lwall<<", Front Wall: "<<Fwall<<", Right Wall: "<<Rwall<<endl;
}