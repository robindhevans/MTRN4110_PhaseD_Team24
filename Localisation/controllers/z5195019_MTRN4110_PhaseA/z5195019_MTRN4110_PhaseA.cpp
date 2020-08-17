//z5195019 PhaseA Epuck Controller

#include <webots/Robot.hpp>
// Added a new include file
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/keyboard.hpp>
#include <webots/Display.hpp>
#include <fstream>
#include <iostream>
#include <string.h>
#include <vector>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define ROTATE_STEP 2.2227
//90 deg arclength dvided by radius of wheels
#define TILE_STEP 8.25
//tile width divided by radius
#define TOLERANCE 0.005
#define PATH_PLAN_FILE_NAME "../../PathPlan.txt"
#define DISP_COR_OFFSET 25
#define DISP_CELL_STEP 50
#define DISP_WIDTH 450
#define DISP_HEIGHT 250
#define MAP_COLS 9
#define MAP_ROWS 5

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

enum heading {North = 0, East, South, West};

typedef struct walldata walldata;
struct walldata {
  int hwalls[MAP_ROWS + 1][MAP_COLS];
  int vwalls[MAP_ROWS][MAP_COLS + 1];
  heading my_heading;
    //North=0 - East=1 - South=2 - West=3
  int row;
  int col;
  };
  
typedef struct robotsensors robotsensors;
struct robotsensors {
  DistanceSensor *front_ds;
  DistanceSensor *right_ds;
  DistanceSensor *left_ds;
  DistanceSensor *back_ds;
  }; 
  
struct walls_detected {
  bool N = true;
  bool E = true;
  bool S = true;
  bool W = true;
  } robot_walls, map_walls;
//struct robot_walls holds last reading from sensors, directions are from robot
  //reference frame
  //struct map_walls holds last wall directions in map reference frame.

//For exploration
void checkWalls (robotsensors sensors, walldata &walls);
void draw_map (walldata walls, Display *displays);
//For localisation
void translate_NESW(heading my_heading, walls_detected &robot_walls, walls_detected &map_walls)
void check_walls_loc(robotsensors sensors, walls_detected &robot_walls);
void floodfill(walldata &walls, int &floodfill_matrix);

int main(int argc, char **argv) {
  walldata walls;
  robotsensors sensors;
  
  //Initialise arrays to all 1's
  for(int x = 0; x < MAP_COLS; x++){
    for(int y = 0; y < MAP_ROWS+1; y++){
      walls.hwalls[y][x] = 1;
    }
  } 
  for(int x = 0; x < MAP_COLS+1; x++){
    for(int y = 0; y < MAP_ROWS; y++){
      walls.vwalls[y][x] = 1;
    }
  } 
  
  double l_position = 0.0;
  double r_position = 0.0;
  
  double l_encoder_pos = 0.0;
  double r_encoder_pos = 0.0;
  
  double FrontDis = 0.0;
  double RightDis = 0.0;
  double LeftDis = 0.0;
  
  //initialise robot position
  walls.row = 0;
  walls.col = 0;
  walls.heading = South;
  
  //initialise robot
  Robot *robot = new Robot();
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  
  Camera *front_camera = robot->getCamera("camera");
  cout << "Camera width: " << front_camera->getWidth() << " - height: " << front_camera->getHeight() << endl;
  
  Keyboard *keyboard = robot->getKeyboard();
  Display *display = robot->getDisplay("display");
  display->setFont("Arial", 30 , 1);
  cout << "Display width: " << display->getWidth() << " - height: " << display->getHeight() << endl;

  sensors.front_ds = robot->getDistanceSensor("F_DS");
  sensors.right_ds = robot->getDistanceSensor("R_DS");
  sensors.left_ds = robot->getDistanceSensor("L_DS");
  sensors.back_ds = robot->getDistanceSensor("B_DS");  
  
  PositionSensor *left_en = robot->getPositionSensor("left wheel sensor");
  PositionSensor *right_en = robot->getPositionSensor("right wheel sensor");
  
  sensors.front_ds->enable(TIME_STEP);
  sensors.right_ds->enable(TIME_STEP);
  sensors.left_ds->enable(TIME_STEP);
  sensors.back_ds->enable(TIME_STEP);
  
  left_en->enable(TIME_STEP);
  right_en->enable(TIME_STEP);
  front_camera->enable(TIME_STEP);
  keyboard->enable(TIME_STEP);
  display->attachCamera(front_camera);

  robot->step(TIME_STEP);  
  
  leftMotor->setVelocity(0.8 * MAX_SPEED);
  rightMotor->setVelocity(0.8 * MAX_SPEED);
  
  // read initial step before entering loop.
  FrontDis = sensors.front_ds->getValue();
  RightDis = sensors.right_ds->getValue();
  LeftDis = sensors.left_ds->getValue();
  
  checkWalls(sensors, walls);
  draw_map(walls, display);
  
  while (robot->step(TIME_STEP) != -1){
  
    int key = keyboard->getKey();
    int step_taken = 0;
      
    //Start forward instruction
    if (key == keyboard->UP){
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
        
        FrontDis = sensors.front_ds->getValue();
        RightDis = sensors.right_ds->getValue();
        LeftDis = sensors.left_ds->getValue();
      }
      //adjust rows and cols value
      switch (walls.heading){
        case North: //NORTH
          walls.row -= 1;
          break;
        case East: //EAST
          walls.col += 1;
          break;
        case South: //SOUTH
          walls.row += 1;
          break;
        case West: //WEST
          walls.col -= 1;
          break;
      }
      step_taken = 1;
    }
    //End forward instruction
    
    //start left rotation instruction
    else if (key == keyboard->LEFT){
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
        
        FrontDis = sensors.front_ds->getValue();
        RightDis = sensors.right_ds->getValue();
        LeftDis = sensors.left_ds->getValue();

      }
      //adjust heading
      switch (walls.heading){
        case (North):
          walls.heading = West;
          break;
        case (East):
          walls.heading = North;
          break;
        case (South):
          walls.heading = East;
          break;
        case (West):
          walls.heading = South;
          break;
      }
      step_taken = 1;
    }
    //End left rotate instruction
    
    //start right rotate instruction
    else if (key == keyboard->RIGHT){
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
        
        FrontDis = sensors.front_ds->getValue();
        RightDis = sensors.right_ds->getValue();
        LeftDis = sensors.left_ds->getValue();
      }
      //adjust heading
      switch (walls.heading){
        case North:
          walls.heading = East;
          break;
        case East:
          walls.heading = South;
          break;
        case South:
          walls.heading = West;
          break;
        case West:
          walls.heading = North;
          break;
      }
      step_taken = 1;
    }
    
    if(step_taken == 1){
      //print step information
      checkWalls(sensors, walls);
      draw_map(walls, display);
      cout << "Horizontal wall array" << endl;
      for(int y = 0; y < MAP_ROWS+1; y++){
        for(int x = 0; x < MAP_COLS; x++){
          cout << walls.hwalls[y][x] << ' '; 
        }
        cout << endl;
      } 
      cout << "Vertical wall array" << endl;
      for(int y = 0; y < MAP_ROWS; y++){
        for(int x = 0; x < MAP_COLS+1; x++){
          cout << walls.vwalls[y][x] << ' ';
        }
        cout << endl;
      } 
    cout << "coords: [" << walls.row<<","<<walls.col<<"] heading: "<<walls.heading <<endl;
    }
    
  }

  cout << "Done - Path plan Executed" << endl;
  
  cout << "The following executes assuming robot is in unknown ";
  cout << " location but heading is stored in KNOWN_HEADING variable" << endl;
  // define food-fill matrix
  int floodfill_matrix[MAP_ROWS][MAP_COLS];
  //initialise floodfill matrix
  for(int i = 0; i < 5; i++) {
    for(int j = 0; j < 9; j++) {
      floodfill_matrix[i][j] = 45;
      //cout << matrix[i][j] << ' ';
    }
    //cout << endl;
  }
  floodfill(walls, floodfill_matrix);
  //setup paths vector
  vector<vector<int>> paths;
  //setup heading
  heading my_heading = KNOWN_HEADING;
  //check surrounding walls
  check_walls_loc(robotsensors sensors, robot_walls);
  // translate to map heading
  translate_NESW(my_heading, robot_walls, map_walls);
  //cout << map_walls.N << map_walls.E << map_walls.S << map_walls.W << endl;
  delete robot;

  return 0;

}

void checkWalls (robotsensors sensors, walldata &walls){  

  //cout << "sensors: F,R,B,L "<< sensors.front_ds->getValue()<< ' ';
  //cout << sensors.right_ds->getValue() << ' ';
  //cout << sensors.back_ds->getValue() << ' ';
  //cout << sensors.left_ds->getValue() << endl;
  if (walls.heading == North){ //NORTH
    if(sensors.front_ds->getValue() < 0.85){
      walls.hwalls[walls.row][walls.col] = 0;
    }
    if(sensors.right_ds->getValue() < 0.85){
      walls.vwalls[walls.row][walls.col+1] = 0;
    }
    if(sensors.back_ds->getValue() < 0.85){
      walls.hwalls[walls.row+1][walls.col] = 0;
    }
    if(sensors.left_ds->getValue() < 0.85){
      walls.vwalls[walls.row][walls.col] = 0;
    }
  }   
  if (walls.heading == East){ //EAST
    if(sensors.front_ds->getValue() < 0.85){
      walls.vwalls[walls.row][walls.col+1] = 0;
    }
    if(sensors.right_ds->getValue() < 0.85){
      walls.hwalls[walls.row+1][walls.col] = 0;
    }
    if(sensors.back_ds->getValue() < 0.85){
      walls.vwalls[walls.row][walls.col] = 0;
    }
    if(sensors.left_ds->getValue() < 0.85){
      walls.hwalls[walls.row][walls.col] = 0; 
    }
  }   
  if (walls.heading == South){ //SOUTH
    if(sensors.front_ds->getValue() < 0.85){
      walls.hwalls[walls.row+1][walls.col] = 0;
    }
    if(sensors.right_ds->getValue() < 0.85){
      walls.vwalls[walls.row][walls.col] = 0;
    }
    if(sensors.back_ds->getValue() < 0.85){
      walls.hwalls[walls.row][walls.col] = 0;
    }
    if(sensors.left_ds->getValue() < 0.85){
      walls.vwalls[walls.row][walls.col+1] = 0;      
    }
  }
  if (walls.heading == West){ //WEST    
    if(sensors.front_ds->getValue() < 0.85){
      walls.vwalls[walls.row][walls.col] = 0;
    }
    if(sensors.right_ds->getValue() < 0.85){
      walls.hwalls[walls.row][walls.col] = 0;
    }
    if(sensors.back_ds->getValue() < 0.85){
      walls.vwalls[walls.row][walls.col+1] = 0;
    }
    if(sensors.left_ds->getValue() < 0.85){
      walls.hwalls[walls.row+1][walls.col] = 0;     
    }
  } 
}

void draw_map (walldata walls, Display *display){

  //clear display
  display->setAlpha(0);
  display->fillRectangle(0,0,500,300);\
  //set up outisde borders
  display->setAlpha(1);
 

//insert walls
  display->setColor(0xba25b0);
  //draw horizontal walls
  for(int x = 0; x < MAP_COLS; x++){
    for(int y = 1; y < MAP_ROWS; y++){
      if(walls.hwalls[y][x] == 1){
        int xcoord = DISP_COR_OFFSET + (x*DISP_CELL_STEP);
        int ycoord = DISP_COR_OFFSET + (y*DISP_CELL_STEP);
        display->drawLine(xcoord, ycoord, xcoord+DISP_CELL_STEP, ycoord);
      }
    }
  } 
  //draw vertical walls
  for(int x = 1; x < MAP_COLS; x++){
    for(int y = 0; y < MAP_ROWS; y++){
      if(walls.vwalls[y][x] == 1){
        int xcoord = DISP_COR_OFFSET + (x*DISP_CELL_STEP);
        int ycoord = DISP_COR_OFFSET + (y*DISP_CELL_STEP);
        display->drawLine(xcoord, ycoord, xcoord, ycoord+DISP_CELL_STEP);
      }
    }
  } 
  
  int rowpos = DISP_COR_OFFSET + (walls.row * DISP_CELL_STEP);
  int colpos = DISP_COR_OFFSET + (walls.col * DISP_CELL_STEP)+15;
  
  switch (walls.heading){
    case 0: //NORTH
      display->drawText("^", colpos, rowpos+18);
      break;
    case 1: //EAST
      display->drawText(">", colpos, rowpos+10);
      break;
    case 2: //SOUTH
      display->drawText("v", colpos, rowpos+8);
      break;
    case 3: //WEST
      display->drawText("<", colpos, rowpos+8);
      break;
  }
  
  display->setColor(0x1f9e1f);
  display->drawRectangle(DISP_COR_OFFSET,DISP_COR_OFFSET,DISP_WIDTH+2,DISP_HEIGHT+2);
}

void translate_NESW(heading my_heading, walls_detected &robot_walls, walls_detected &map_walls) {
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

void check_walls_loc(robotsensors sensors, walls_detected &robot_walls) {
  if(sensors.front_ds->getValue() < 0.85) {
      robot_walls->N = false;
  }
  if(sensors.right_ds->getValue() < 0.85) {
      robot_walls->E = false;
  }
  if(sensors.back_ds->getValue() < 0.85) {
      robot_walls->S = false;
  }
  if(sensors.left_ds->getValue() < 0.85) {
      robot_walls->W = false;
  }
}

void floodfill(walldata &walls, int &floodfill_matrix) {
//begin flood-fill algorithm
  int current_explored_val = 0;
  bool maze_val_changed = true;
  while(maze_val_changed == true) {
    maze_val_changed = FALSE;
    for(int i = 0; i < 5; i++) {
      for(int j = 0; j < 9; j++) {
        //check if current cell has curent value
        if (floodfill_matrix[i][j] == current_explored_val) {
          //check north for wall and unexplored
          if (hwalls[i][j] == FALSE) {
            if(floodfill_matrix[i-1][j] == 45) {
              floodfill_matrix[i-1][j] = current_explored_val + 1;
              maze_val_changed = TRUE;
            }
          }
          //check south for wall and unexplored
          if (hwalls[i+1][j] == FALSE) {
            if(floodfill_matrix[i+1][j] == 45) {
              floodfill_matrix[i+1][j] = current_explored_val + 1;
              maze_val_changed = TRUE;
            }
          }
          //check west for wall and unexplored
          if (vwalls[i][j] == FALSE) {
            if(floodfill_matrix[i][j-1] == 45) {
              floodfill_matrix[i][j-1] = current_explored_val + 1;
              maze_val_changed = TRUE;
            }
          }
          //check east for wall and unexplored
          if (vwalls[i][j+1] == FALSE) {
            if(floodfill_matrix[i][j+1] == 45) {
              floodfill_matrix[i][j+1] = current_explored_val + 1;
              maze_val_changed = TRUE;
            }
          }
        }
      }
    }
  //increment explored value (for use later or remove if not needed)
  current_explored_val++;
  }
 }