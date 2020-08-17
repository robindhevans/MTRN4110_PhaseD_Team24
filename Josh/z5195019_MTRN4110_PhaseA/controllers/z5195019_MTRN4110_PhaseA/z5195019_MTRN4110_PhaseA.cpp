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


void checkWalls (char& Lwall, char& Fwall, char& Rwall, double LeftDis, double FrontDis, double RightDis);

int main(int argc, char **argv) {
  //variables
  char pathPlan[50];
  
  double l_position = 0.0;
  double r_position = 0.0;
  
  double l_encoder_pos = 0.0;
  double r_encoder_pos = 0.0;
  
  double FrontDis = 0.0;
  double RightDis = 0.0;
  double LeftDis = 0.0;
  
  int row = 0;
  int col = 0;
  char heading = 'N';
  char Lwall = 'N';
  char Fwall = 'N';
  char Rwall = 'N';
  
  //initialise robot
  Robot *robot = new Robot();
  // get a handler to the motors.
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");

  DistanceSensor *front_ds = robot->getDistanceSensor("F_DS");
  DistanceSensor *right_ds = robot->getDistanceSensor("R_DS");
  DistanceSensor *left_ds = robot->getDistanceSensor("L_DS");
  
  PositionSensor *left_en = robot->getPositionSensor("left wheel sensor");
  PositionSensor *right_en = robot->getPositionSensor("right wheel sensor");
  
  front_ds->enable(TIME_STEP);
  right_ds->enable(TIME_STEP);
  left_ds->enable(TIME_STEP);
  left_en->enable(TIME_STEP);
  right_en->enable(TIME_STEP);

  robot->step(TIME_STEP);
  

  //leftMotor->setPosition(8.25);
  //rightMotor->setPosition(8.25);
  leftMotor->setVelocity(0.8 * MAX_SPEED);
  rightMotor->setVelocity(0.8 * MAX_SPEED);
  //leftMotor->setPosition(TILE_STEP);
  //rightMotor->setPosition(TILE_STEP);
  
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
  
  // read initial step before entering loop.
  FrontDis = front_ds->getValue();
  RightDis = right_ds->getValue();
  LeftDis = left_ds->getValue();
  
  checkWalls(Lwall, Fwall, Rwall, LeftDis, FrontDis, RightDis);
  
  row = (int)pathPlan[0]-48;
  col = (int)pathPlan[1]-48;
  heading = pathPlan[2];
  
  cout<<"Step: 00, Row: "<<row<<", Column: "<<col<<", Heading: "<<heading<<", Left Wall: "<<Lwall<<", Front Wall: "<<Fwall<<", Right Wall: "<<Rwall<<endl;
 
  int i = 3;
  int planLength = strlen(pathPlan);

  while (i < planLength && robot->step(TIME_STEP) != -1){
      
    //Start forward instruction
    if (pathPlan[i] == 'F'){
      leftMotor->setVelocity(0.8 * MAX_SPEED);
      rightMotor->setVelocity(0.8 * MAX_SPEED);
        
      leftMotor->setPosition(l_position + TILE_STEP);
      rightMotor->setPosition(r_position + TILE_STEP);
      
      l_position += TILE_STEP;
      r_position += TILE_STEP;
      //read sesors to check position and walls  
      while (l_position-TOLERANCE >= l_encoder_pos && r_position-TOLERANCE >= r_encoder_pos){
        robot->step(TIME_STEP);
        l_encoder_pos = left_en->getValue();
        r_encoder_pos = right_en->getValue();
        
        FrontDis = front_ds->getValue();
        RightDis = right_ds->getValue();
        LeftDis = left_ds->getValue();
      }
      //adjust rows and cols value
      switch (heading){
        case 'S':
          row += 1;
          break;
        case 'N':
          row -= 1;
          break;
        case 'E':
          col += 1;
          break;
        case 'W':
          col -= 1;
          break;
      }
    }
    //End forward instruction
    
    //start left rotation instruction
    else if (pathPlan[i] == 'L'){
      leftMotor->setVelocity(0.4 * MAX_SPEED);
      rightMotor->setVelocity(0.4 * MAX_SPEED);
       
      leftMotor->setPosition(l_position - ROTATE_STEP);
      rightMotor->setPosition(r_position + ROTATE_STEP);
      
      l_position -= ROTATE_STEP;
      r_position += ROTATE_STEP;
      //read sesors to check position and walls  
      while (l_position+TOLERANCE <= l_encoder_pos && r_position-TOLERANCE >= r_encoder_pos){
        robot->step(TIME_STEP);
        l_encoder_pos = left_en->getValue();
        r_encoder_pos = right_en->getValue();
        
        FrontDis = front_ds->getValue();
        RightDis = right_ds->getValue();
        LeftDis = left_ds->getValue();

      }
      //adjust heading
      switch (heading){
        case 'S':
          heading = 'E';
          break;
        case 'N':
          heading = 'W';
          break;
        case 'E':
          heading = 'N';
          break;
        case 'W':
          heading = 'S';
          break;
      }
    }
    //End left rotate instruction
    
    //start right rotate instruction
    else if (pathPlan[i] == 'R'){
      leftMotor->setVelocity(0.4 * MAX_SPEED);
      rightMotor->setVelocity(0.4 * MAX_SPEED);
       
      leftMotor->setPosition(l_position + ROTATE_STEP);
      rightMotor->setPosition(r_position - ROTATE_STEP);
      
      l_position += ROTATE_STEP;
      r_position -= ROTATE_STEP;
      //read sesors to check position and walls  
      while (l_position-TOLERANCE >= l_encoder_pos && r_position+TOLERANCE <= r_encoder_pos){
        robot->step(TIME_STEP);
        l_encoder_pos = left_en->getValue();
        r_encoder_pos = right_en->getValue();
        
        FrontDis = front_ds->getValue();
        RightDis = right_ds->getValue();
        LeftDis = left_ds->getValue();
        //cout << "getting pos values" << endl;
      }
      //adjust heading
      switch (heading){
        case 'S':
          heading = 'W';
          break;
        case 'N':
          heading = 'E';
          break;
        case 'E':
          heading = 'S';
          break;
        case 'W':
          heading = 'N';
          break;
      }
    }
    
    //print step information
    checkWalls(Lwall, Fwall, Rwall, LeftDis, FrontDis, RightDis);
    printf("Step: %02d, ", i-2);
    cout<<"Row: "<<row<<", Column: "<<col<<", Heading: "<<heading;
    cout<<", Left Wall: "<<Lwall<<", Front Wall: "<<Fwall<<", Right Wall: "<<Rwall<<endl;
    i++;
  }

  cout << "Done - Path plan Executed" << endl;

  delete robot;
  pathFile.close();

  return 0;

}



void checkWalls (char& Lwall, char& Fwall, char& Rwall, double LeftDis, double FrontDis, double RightDis){

  //correct wall variables
  if(LeftDis > 0.85){
    Lwall = 'Y';
  }else{
    Lwall = 'N';
  }
  
  if(FrontDis > 0.85){
    Fwall = 'Y';
  }else{ 
    Fwall = 'N';
  }
  
  if(RightDis > 0.85){
    Rwall = 'Y';
  }else{
    Rwall = 'N';
  }
}