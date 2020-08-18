// TheA-maze-ingRacers Epuck controller
// Based off of z5195019's Phase A Epuck Controller (Joshua Purnell)
// with alterations and additions from:
//             - Robin Evans (z5197018)
//             - Dean So (z5204873)

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
#include <algorithm>

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
#define GOAL_ROW MAP_ROWS/2
#define GOAL_COL MAP_COLS/2
#define KNOWN_HEADING South

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

enum heading {North = 0, East, South, West};

typedef struct walldata walldata;
struct walldata {
  int hwalls[MAP_ROWS + 1][MAP_COLS];
  int vwalls[MAP_ROWS][MAP_COLS + 1];
  heading heading_;
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
void checkWalls(robotsensors sensors, walldata &walls);
void draw_map(walldata walls, Display *displays);
//For localisation
void translate_NESW(heading my_heading, walls_detected (&robot_walls), walls_detected (&map_walls));
void check_walls_loc(robotsensors sensors, walls_detected (&robot_walls));
void floodfill(walldata (&walls), int (&floodfill_matrix)[MAP_ROWS][MAP_COLS]);
int get_rc(int id, char setting);
int get_id(int r, int c);
void advance_direction(heading (&direction));
bool compare_walls(walldata (&walls), int row, int col, walls_detected map_walls);
void get_short_path_from_id(int (&floodfill_matrix)[MAP_ROWS][MAP_COLS], int start_id, int goal_id, vector<int> (&temp_path), walldata (&walls));

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
  walls.heading_ = South;
  
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
  
  while (robot->step(TIME_STEP) != -1) {
  
    int key = keyboard->getKey();
    int step_taken = 0;
      
    //Start forward instruction
    if (key == keyboard->UP) {
      leftMotor->setVelocity(0.8 * MAX_SPEED);
      rightMotor->setVelocity(0.8 * MAX_SPEED);
        
      leftMotor->setPosition(l_position + TILE_STEP);
      rightMotor->setPosition(r_position + TILE_STEP);
      
      l_position += TILE_STEP;
      r_position += TILE_STEP;
      //read sesors to check position and walls  
      while (l_position-TOLERANCE >= l_encoder_pos && r_position-TOLERANCE >= r_encoder_pos) {
        robot->step(TIME_STEP);
        l_encoder_pos = left_en->getValue();
        r_encoder_pos = right_en->getValue();
        
        FrontDis = sensors.front_ds->getValue();
        RightDis = sensors.right_ds->getValue();
        LeftDis = sensors.left_ds->getValue();
      }
      //adjust rows and cols value
      switch (walls.heading_) {
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
    else if (key == keyboard->LEFT) {
      leftMotor->setVelocity(0.4 * MAX_SPEED);
      rightMotor->setVelocity(0.4 * MAX_SPEED);
       
      leftMotor->setPosition(l_position - ROTATE_STEP);
      rightMotor->setPosition(r_position + ROTATE_STEP);
      
      l_position -= ROTATE_STEP;
      r_position += ROTATE_STEP;
      //read sesors to check position and walls  
      while (l_position+TOLERANCE <= l_encoder_pos && r_position-TOLERANCE >= r_encoder_pos) {
        robot->step(TIME_STEP);
        l_encoder_pos = left_en->getValue();
        r_encoder_pos = right_en->getValue();
        
        FrontDis = sensors.front_ds->getValue();
        RightDis = sensors.right_ds->getValue();
        LeftDis = sensors.left_ds->getValue();

      }
      //adjust heading
      switch (walls.heading_){
        case (North):
          walls.heading_ = West;
          break;
        case (East):
          walls.heading_ = North;
          break;
        case (South):
          walls.heading_ = East;
          break;
        case (West):
          walls.heading_ = South;
          break;
      }
      step_taken = 1;
    }
    //End left rotate instruction
    
    //start right rotate instruction
    else if (key == keyboard->RIGHT) {
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
      switch (walls.heading_){
        case North:
          walls.heading_ = East;
          break;
        case East:
          walls.heading_ = South;
          break;
        case South:
          walls.heading_ = West;
          break;
        case West:
          walls.heading_ = North;
          break;
      }
      step_taken = 1;
    } else if (key == keyboard->END) {
      // manual skip to end but robot should exit when the 
      // entire maze has been explored under normal operation
      break;
    }
    
    if (step_taken == 1){
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
      cout << "coords: [" << walls.row<<","<< walls.col <<"] heading: "<< walls.heading_ <<endl;
    }
    
  }
  
  // ----------------------------- LOCALISATION ----------------------------- //
  cout << "The following executes assuming robot is in unknown";
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
  // initialise goal as 0

  
  // testing walls to skip exploration part
  int temphwall[MAP_ROWS+1][MAP_COLS] = {{1, 1, 1, 1, 1, 1, 1, 1, 1},
                                         {0, 1, 0, 0, 0, 0, 1, 0, 0},
                                         {1, 0, 0, 0, 1, 1, 0, 1, 0},
                                         {0, 0, 1, 0, 1, 1, 1, 0, 0},
                                         {0, 0, 1, 0, 0, 1, 0, 1, 0},
                                         {1, 1, 1, 1, 1, 1, 1, 1, 1}};
  int tempvwall[MAP_ROWS][MAP_COLS+1] = {{1, 0, 0, 0, 1, 0, 0, 0, 0, 1}, 
                                         {1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
                                         {1, 0, 1, 1, 1, 0, 0, 0, 0, 1},
                                         {1, 1, 0, 0, 0, 0, 1, 0, 0, 1},
                                         {1, 1, 0, 0, 1, 0, 0, 1, 0, 1}};
  
  for (int i = 0; i < MAP_ROWS+1; ++i) {
    for (int j = 0; j < MAP_COLS; ++j) {
      walls.hwalls[i][j] = temphwall[i][j];
    }
  }
  for (int i = 0; i < MAP_ROWS; ++i) {
    for (int j = 0; j < MAP_COLS+1; ++j) {
      walls.vwalls[i][j] = tempvwall[i][j];
    }
  }
  
  floodfill_matrix[GOAL_ROW][GOAL_COL] = 0;
  for (auto& e : walls.hwalls) {
    for (auto& f : e) {
        std::cout << f << " ";
    }
    cout << endl;
  }
  
  floodfill(walls, floodfill_matrix);
  for (auto& e : floodfill_matrix) {
    for (auto& f : e) {
        if (f < 10) cout << '0';
        std::cout << f << " ";
    }
    cout << endl;
  }
  //setup paths vector
  vector<vector<int>> paths;
  // possible intial locations matrix
  vector<int> potential_cells {};
  //vector to hold current path
  vector<int> current_path {};

  //setup heading
  heading my_heading = KNOWN_HEADING;
  //check surrounding walls
  robot->step(TIME_STEP);
  check_walls_loc(sensors, robot_walls); // robot_walls has current robot sensor data
  // translate to robot_walls to reference frame of map in terms of NSEW directions
  // - have to do this each time walls are checked to transform the frame to an object frame
  translate_NESW(my_heading, robot_walls, map_walls);
  cout << map_walls.N << map_walls.E << map_walls.S << map_walls.W << endl;
  
  // search flood fill for possible starting floodfill_matrix
  // currently pushing the possible short paths into paths vector
  for (int r = 0; r < MAP_ROWS; ++r) {
    for (int c = 0; c < MAP_COLS; ++c) {
      if (floodfill_matrix[r][c] < 45) {
        if (compare_walls(walls, r, c, map_walls) == true) {
          //potential_cells.push_back(get_id(r, c));
          cout << r << c << endl;
          vector<int> temp_path;
          temp_path.push_back(get_id(r,c));
          //get_short_path_from_id(floodfill_matrix, get_id(r, c), get_id(GOAL_ROW, GOAL_COL), temp_path, walls);
          paths.push_back(temp_path);
        }
      }
    }
  }
  //find id of potential positions that have the smallest floodfill value
  //that ISNT the goal
  int smallestflood = 45;
  int current_path_id;
  for (int i = 0; i < paths.size(); i++) {
    int r = get_rc(paths[i][0], 'r');
    int c = get_rc(paths[i][0], 'c');
    if (r != GOAL_ROW || c != GOAL_COL) {
      if (floodfill_matrix[r][c] < smallestflood) {
        smallestflood = floodfill_matrix[r][c];
        current_path_id = paths[i][0];
        cout << smallestflood << endl;
      }
    }
  }
  //generate path from start closest to goal
  get_short_path_from_id(floodfill_matrix, current_path_id, get_id(GOAL_ROW, GOAL_COL), current_path, walls);
  
  /*cout << "potential cells ";
  for (auto& e : potential_cells) {
    cout << e << " ";
  }
  cout << endl;
  */
  for (auto& e : paths) {
    cout << "[";
    for (auto& f : e) {
        std::cout << f << " ";
    }
    cout << "]" << endl;
  }
  for (auto& e : current_path) {
    cout << e << " ";
  }
  cout << endl;
  /*
  vector<int> cur_cells {};
  vector<int> prev_cells {};
  int goal_id = get_id(GOAL_ROW, GOAL_COL));
  int from_goal = goal_id;
  copy(potential_cells.begin(), potential_cells.end(), back_inserter prev_cells));
  // localisation algorithm
  while (true) {
    if  prev_cells.size() > 1) {
      if (find prev_cells.begin(), prev_cells.end(), goal_id) != prev_cells.end()) {
        // must store the last goal connected cell in from_goal
      } else {
        // goal is not a potential cell

      }
    } else {
      break;
    }
  }
 */
  delete robot;
  
  return 0;

}

void checkWalls(robotsensors sensors, walldata &walls){  

  //cout << "sensors: F,R,B,L "<< sensors.front_ds->getValue()<< ' ';
  //cout << sensors.right_ds->getValue() << ' ';
  //cout << sensors.back_ds->getValue() << ' ';
  //cout << sensors.left_ds->getValue() << endl;
  if (walls.heading_ == North){ //NORTH
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
  if (walls.heading_ == East){ //EAST
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
  if (walls.heading_ == South){ //SOUTH
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
  if (walls.heading_ == West){ //WEST    
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

void draw_map(walldata walls, Display *display){

  //clear display
  display->setAlpha(0);
  display->fillRectangle(0,0,500,300);
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
  
  switch (walls.heading_){
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

void translate_NESW(heading my_heading, walls_detected (&robot_walls), walls_detected (&map_walls)) {
  //translates wall locations from robot reference frame to map reference frame
  //use map_walls when determining location
  switch (my_heading) {
    case(North):
      map_walls.N = robot_walls.N;
      map_walls.E = robot_walls.E;
      map_walls.S = robot_walls.S;
      map_walls.W = robot_walls.W;
      break;
    case(East):
      map_walls.N = robot_walls.W;
      map_walls.E = robot_walls.N;
      map_walls.S = robot_walls.E;
      map_walls.W = robot_walls.S;
      break;
    case(South):
      map_walls.N = robot_walls.S;
      map_walls.E = robot_walls.W;
      map_walls.S = robot_walls.N;
      map_walls.W = robot_walls.E;
      break;
    case(West):
      map_walls.N = robot_walls.E;
      map_walls.E = robot_walls.S;
      map_walls.S = robot_walls.W;
      map_walls.W = robot_walls.N;
  }
}

void check_walls_loc(robotsensors sensors, walls_detected (&robot_walls)) {
  if(sensors.front_ds->getValue() < 0.85) {
      robot_walls.N = false;
  }
  if(sensors.right_ds->getValue() < 0.85) {
      robot_walls.E = false;
  }
  if(sensors.back_ds->getValue() < 0.85) {
      robot_walls.S = false;
  }
  if(sensors.left_ds->getValue() < 0.85) {
      robot_walls.W = false;
  }
}

void floodfill(walldata (&walls), int (&floodfill_matrix)[MAP_ROWS][MAP_COLS]) {
  // flood fill algorithm
  int cur_explored_cell = 0;
  bool maze_value_changed = true;
  heading dir_check = North;
  int N = 45;
  // flood fill loop
  while (maze_value_changed != false) {
    maze_value_changed = false;
    for (int r = 0; r < MAP_ROWS; ++r) {
      for (int c = 0; c < MAP_COLS; ++c) {
        if (floodfill_matrix[r][c] == cur_explored_cell) {
          dir_check = North;
          //dir_mod = 0;
          for (int d = 0; d < 4; ++d) {
            if (dir_check == North && r != 0 && walls.hwalls[r][c] == 0) {
              // if searching north but at top row dont need to update anything
              if (floodfill_matrix[r-1][c] == N) {
                floodfill_matrix[r-1][c] = floodfill_matrix[r][c] + 1;
                maze_value_changed = true;
              }

            } else if (dir_check == East && c != MAP_COLS - 1 && walls.vwalls[r][c+1] == 0) {
              // if searching east but at right most col dont need to update anything
              if (floodfill_matrix[r][c+1] == N) {
                floodfill_matrix[r][c+1] = floodfill_matrix[r][c] + 1;
                maze_value_changed = true;
              }
            } else if (dir_check == South && r != MAP_ROWS - 1 && walls.hwalls[r+1][c] == 0) {
              // if searching south but at bottom row dont need to update anything
              if (floodfill_matrix[r+1][c] == N) {
                floodfill_matrix[r+1][c] = floodfill_matrix[r][c] + 1;
                maze_value_changed = true;
              }
            } else if (dir_check == West && c != 0 && walls.vwalls[r][c] == 0) {
              // if searching west but at left most col dont need to update anything
              if (floodfill_matrix[r][c-1] == N) {
                floodfill_matrix[r][c-1] = floodfill_matrix[r][c] + 1;
                maze_value_changed = true;
              }
            }
            advance_direction(dir_check);
          }
        }
      }
    }
      ++cur_explored_cell;
  } 
}
 
int get_rc(int id, char setting) {
  int val = 0;
  if (setting == 'r') {
    val = id / 9;
  } else if (setting == 'c') {
    val = id % 9;
  }
  return val;
}

int get_id(int r, int c) {
  return (r * 9) + c;
}

void advance_direction(heading (&direction)) {
  //rotate direction clockwise
  switch (direction) {
    case North:
      direction = East;
      break;
    case East:
      direction = South;
      break;
    case South:
      direction = West;
      break;
    case West:
      direction = North;
      break;
  }
}

bool compare_walls(walldata (&walls), int row, int col, walls_detected map_walls) {
  // for each direction check if that wall matches with the map_walls 
  // if at least one of the walls dont match, then we know its not an potential starting position
  bool wall_check = true;
  //begin at north
  heading dir_check = North;
  //for all directions check if current location matches
  for (int d = 0; d < 4; ++d) {
    if (dir_check == North) {
      if (walls.hwalls[row][col] != map_walls.N) {
        wall_check = false;
      }
    } else if (dir_check == East) {
      if (walls.vwalls[row][col+1] != map_walls.E) {
        wall_check = false;
      }
    } else if (dir_check == South) {
      if (walls.hwalls[row+1][col] != map_walls.S) {
        wall_check = false;
      }
    } else if (dir_check == West) {
        if (walls.vwalls[row][col] != map_walls.W) {
        wall_check = false;
      }
    }
    //check next direction
    advance_direction(dir_check);
  }
  //return bool whether walls match
  return wall_check;
}

void get_short_path_from_id(int (&floodfill_matrix)[MAP_ROWS][MAP_COLS], int start_id, int goal_id, vector<int> (&temp_path), walldata (&walls)) {
  // grab the shortest path from a start id to goal id
  int cur_id = start_id;
  heading cur_heading = KNOWN_HEADING;
  int cur_flood = 0;
  heading dir_check = North;
  while (cur_id != goal_id) {
    int r = get_rc(cur_id, 'r');
    int c = get_rc(cur_id, 'c');
    // make the first check in the direction of the robot to optimise turns
    // essentially, robot goes straight when it can since the first direction
    // checked is the previous heading of the robot...
    dir_check = cur_heading;
    cur_flood = floodfill_matrix[r][c];
    // dont need to check if the value is < 45 since we check for a wall
    // if theres a wall its not checked already...
    for (int d = 0; d < 4; ++d) {
      // checks performed for each direction:
      //        - edge cases for r and c
      //        - no wall in that direction
      //        - next cell in that direction is 1 less than previous cell
      
      if (dir_check == North && r != 0 && walls.hwalls[r][c] == 0 && floodfill_matrix[r-1][c] == cur_flood-1) {
        cur_id = get_id(r-1, c);
        cur_flood = floodfill_matrix[r-1][c];
        cur_heading = North;
        temp_path.push_back(cur_id);
      } else if (dir_check == East && c != MAP_COLS - 1 && walls.vwalls[r][c+1] == 0 && floodfill_matrix[r][c+1] == cur_flood-1) {
        cur_id = get_id(r, c+1);
        cur_flood = floodfill_matrix[r][c+1];
        cur_heading = East;
        temp_path.push_back(cur_id);
      } else if (dir_check == South && r != MAP_ROWS - 1 && walls.hwalls[r+1][c] == 0 && floodfill_matrix[r+1][c] == cur_flood-1) {
        cur_id = get_id(r+1, c);
        cur_flood = floodfill_matrix[r+1][c];
        cur_heading = South;
        temp_path.push_back(cur_id);
      } else if (dir_check == West && c != 0 && walls.vwalls[r][c] == 0 && floodfill_matrix[r][c-1] == cur_flood-1) {
        cur_id = get_id(r, c-1);
        cur_flood = floodfill_matrix[r][c-1];
        cur_heading = West;
        temp_path.push_back(cur_id);
      }
      advance_direction(dir_check);
    }
  }
}