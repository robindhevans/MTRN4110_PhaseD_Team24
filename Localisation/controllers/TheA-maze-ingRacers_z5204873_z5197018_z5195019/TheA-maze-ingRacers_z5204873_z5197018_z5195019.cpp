// TheA-maze-ingRacers Epuck controller
// Based off of z5195019's Phase A Epuck Controller (Joshua Purnell)
// with alterations and additions from:
//             - Robin Evans (z5197018)
//             - Dean So (z5204873)
//             - Joshua Purnell (z5195019)

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
#define get_path_instruct_FILE_NAME "../../PathPlan.txt"
#define MAP_FILE_NAME "../../Map.txt"
#define DISP_COR_OFFSET 25
#define DISP_CELL_STEP 50
#define DISP_WIDTH 450
#define DISP_HEIGHT 250
#define MAP_COLS 9
#define MAP_ROWS 5
#define GOAL_ROW MAP_ROWS/2
#define GOAL_COL MAP_COLS/2
#define KNOWN_HEADING South
#define HORIZONTAL_WALL "---"
#define VERTICAL_WALL "|"
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
void draw_progress(walldata walls, Display *displays);

//For localisation
void translate_NESW(heading my_heading, walls_detected (&robot_walls), walls_detected (&map_walls));
void check_walls_loc(Robot *robot, robotsensors sensors, walls_detected (&robot_walls));
void floodfill(walldata (&walls), int (&floodfill_matrix)[MAP_ROWS][MAP_COLS]);
int get_rc(int id, char setting);
int get_id(int r, int c);
void advance_direction(heading (&direction));
bool compare_walls(walldata (&walls), int row, int col, walls_detected map_walls);
void get_short_path_from_id(int (&floodfill_matrix)[MAP_ROWS][MAP_COLS], int start_id, int goal_id, vector<int> (&temp_path), walldata (&walls));
string get_path_instruct(int start_id, int goal_id, heading init_heading, vector<int> path, int hWall[MAP_ROWS+1][MAP_COLS], int vWall[MAP_ROWS][MAP_COLS+1]);
int dir_num(heading direction);
int dir_dif(heading a, heading b);
heading dir_trans(heading dir_num, int dir_dif);
void robot_follow_steps(char instruct, Robot* robot, robotsensors (&sensors), 
                        Motor* leftMotor, Motor* rightMotor, 
                        PositionSensor *left_en, PositionSensor *right_en);
void draw_potentials(walldata walls, Display *display, vector<vector<int>> potentials, bool found_state);
void scan_wall_values(walldata (&walls), std::vector<std::string> map);
int main(int argc, char **argv) {
  walldata walls;
  robotsensors sensors;
  
  //Initialise arrays to all 2's represent unknown walls
  for(int x = 0; x < MAP_COLS; x++){
    for(int y = 0; y < MAP_ROWS+1; y++){
      walls.hwalls[y][x] = 2;
    }
  } 
  for(int x = 0; x < MAP_COLS+1; x++){
    for(int y = 0; y < MAP_ROWS; y++){
      walls.vwalls[y][x] = 2;
    }
  } 
  
  std::string line;
  std::vector<std::string> map;
  std::ifstream map_txt (MAP_FILE_NAME);
  
  if (map_txt.is_open()) {
    while (getline(map_txt, line)) {
      map.push_back(line);
    }
    map_txt.close();
  } else {
    std::cout << "Unable to open file" << std::endl;
  }

  double l_position = 0.0;
  double r_position = 0.0;
  
  double l_encoder_pos = 0.0;
  double r_encoder_pos = 0.0;
  
  double FrontDis = 0.0;
  double RightDis = 0.0;
  double LeftDis = 0.0;
  
  bool force_exit_exploration = false;

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
  draw_progress(walls, display);
  
  while (robot->step(TIME_STEP) != -1) {
  
    int key = keyboard->getKey();
    int step_taken = 0;
      
    //Start forward instruction
    if (key == keyboard->UP) {
      robot_follow_steps('F', robot, sensors, leftMotor, rightMotor, left_en, right_en);
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
      robot_follow_steps('L', robot, sensors, leftMotor, rightMotor, left_en, right_en);
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
      robot_follow_steps('R', robot, sensors, leftMotor, rightMotor, left_en, right_en);
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
      force_exit_exploration = true;
      break;
    }
    
    if (step_taken == 1){
      //print step information
      checkWalls(sensors, walls);
      draw_map(walls, display);
      draw_progress(walls, display);
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
  // original world
  if (force_exit_exploration == true) {
    // hardcoded walls for skipping exploration part OR use map.txt and read
    // using scan wall values
    scan_wall_values(walls, map);
    // int temphwall[MAP_ROWS+1][MAP_COLS] = {{1, 1, 1, 1, 1, 1, 1, 1, 1},
    //                                     {0, 1, 0, 0, 0, 0, 1, 0, 0},
    //                                     {1, 0, 0, 0, 1, 1, 0, 1, 0},
    //                                     {0, 0, 1, 0, 1, 1, 1, 0, 0},
    //                                     {0, 0, 1, 0, 0, 1, 0, 1, 0},
    //                                     {1, 1, 1, 1, 1, 1, 1, 1, 1}};
    // int tempvwall[MAP_ROWS][MAP_COLS+1] = {{1, 0, 0, 0, 1, 0, 0, 0, 0, 1}, 
    //                                       {1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
    //                                       {1, 0, 1, 1, 1, 0, 0, 0, 0, 1},
    //                                       {1, 1, 0, 0, 0, 0, 1, 0, 0, 1},
    //                                       {1, 1, 0, 0, 1, 0, 0, 1, 0, 1}};
    // testworld1            
    // int temphwall[MAP_ROWS+1][MAP_COLS] = {{1, 1, 1, 1, 1, 1, 1, 1, 1},
    //                                       {0, 0, 1, 0, 0, 1, 0, 0, 0},
    //                                       {0, 1, 0, 0, 1, 0, 0, 0, 0},
    //                                       {0, 1, 0, 0, 1, 0, 0, 0, 0},
    //                                       {0, 0, 0, 0, 0, 0, 0, 0, 0},
    //                                       {1, 1, 1, 1, 1, 1, 1, 1, 1}};
    // int tempvwall[MAP_ROWS][MAP_COLS+1] = {{1, 0, 0, 0, 0, 0, 0, 0, 0, 1}, 
    //                                       {1, 0, 1, 1, 0, 1, 1, 0, 0, 1},
    //                                       {1, 1, 0, 1, 1, 0, 1, 0, 0, 1},
    //                                       {1, 0, 1, 1, 0, 1, 1, 0, 0, 1},
    //                                       {1, 0, 0, 0, 0, 0, 0, 0, 0, 1}};
    // for (int i = 0; i < MAP_ROWS+1; ++i) {
    //   for (int j = 0; j < MAP_COLS; ++j) {
    //     walls.hwalls[i][j] = temphwall[i][j];
    //   }
    // }
    // for (int i = 0; i < MAP_ROWS; ++i) {
    //   for (int j = 0; j < MAP_COLS+1; ++j) {
    //     walls.vwalls[i][j] = tempvwall[i][j];
    //   }
    // }
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
  //setup paths vector to hold visited cells and potential locations
  vector<vector<int>> paths;
  //vector to hold current path
  vector<int> current_path {};
  //setup heading
  heading my_heading = KNOWN_HEADING;
  //check surrounding walls
  //robot->step(TIME_STEP);
  check_walls_loc(robot, sensors, robot_walls); // robot_walls has current robot sensor data
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
          //cout << r << c << endl;
          vector<int> temp_path;
          temp_path.push_back(get_id(r,c));
          paths.push_back(temp_path);
        }
      }
    }
  }
  //find id of potential positions that have the smallest floodfill value
  //that ISNT the goal
  int smallestflood = 45;
  int current_path_id = 0;
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
  // NOTE: get_short_path_from_id DOES NOT GET PATH WITH LEAST TURNS
  get_short_path_from_id(floodfill_matrix, current_path_id, get_id(GOAL_ROW, GOAL_COL), current_path, walls);
  cout << "initial paths" << endl;
  for (auto& e : paths) {
    cout << "[";
    for (auto& f : e) {
        std::cout << f;
    }
    cout << "]" << endl;
  }
  cout << "current path ";
  for (auto& e : current_path) {
    cout << e << " ";
  }
  cout << endl;
  //generate instructions
  cout << "current_path_id " << current_path_id << endl;
  string robot_instruct = get_path_instruct(current_path_id, get_id(GOAL_ROW,GOAL_COL), my_heading, current_path, walls.hwalls, walls.vwalls);
  cout << "intial instructions: " << robot_instruct << endl;
  // robot_follow_steps(robot_instruct, robot, sensors, walls, leftMotor,
  //                    rightMotor, left_en, right_en, l_position, r_position, 
  //                    l_encoder_pos, r_encoder_pos);
  /*
  vector<int> cur_path                current_path
  Vector<vector<int>> path            path
  Path: get potential cells id           
  Cur_path : Generate new shortest path from end of a non goal starting path and follow 
  while (true): 
    if  number of vectors in path vector > 1 
      take cur_path next valid step (update heading here, maybe make a function that moves the robot as needed in a certain direction regardless of heading?) 
      Add the relative position change from the “next valid step” to each position in path 
      Compare map_walls with walls in each relatively changed position in path (check the last position pushed onto each path vector) 
      Remove paths from path vector that don’t match the wall comparison test 
      If at end of cur_path and number of vectors in path vector > 1 
        Cur_path: generate new shortest path from end of a non goal ending path to goal 
    else break 
  */
  bool found_state = false;
  string::iterator next_step = robot_instruct.begin();
  bool current_path_destroyed = false;
  while (true) {
  //check to see if more than one path left
    if (paths.size() > 1) {
      //check to see if at end of current path instructions
      if(next_step != robot_instruct.end()) {
        //get next instruction from current path
        next_step = robot_instruct.begin();
        //implement instruction
        cout << "Turning ========= " << *next_step << endl;
        robot->step(TIME_STEP);
        robot_follow_steps(*next_step, robot, sensors, leftMotor,
                           rightMotor, left_en, right_en);
        robot->step(TIME_STEP);

        if (*next_step == 'F') {
          cout << "entered here" << endl;
          //if robot has moved forward, check walls
          check_walls_loc(robot, sensors, robot_walls);
          cout << "entered here2" << endl;

          cout <<"Robot Walls(FRBL): "<< robot_walls.N << robot_walls.E << robot_walls.S << robot_walls.W << endl;
          //put corresponding world reference walls into map_walls
          translate_NESW(my_heading, robot_walls, map_walls);
            cout << "entered here3" << endl;

          cout <<"Map Walls(NESW): "<< map_walls.N << map_walls.E << map_walls.S << map_walls.W << endl;
          //update paths vector vectors (could be in a function for neatness)
          // each time loop goes through, the paths.end() gets reevaluated each time
          for (vector<vector<int>>::iterator it = paths.begin(); it != paths.end();) {
            //get path vector
            vector<int> temp_path = *it;
            cout << "entered here4" << endl;
            //cout << temp_path[0] << endl;
            //get latest potential location
            int prev_cell_id = temp_path.back();
            cout << "prev_cell_id = " << prev_cell_id << endl;
            int r = 0;
            int c = 0;
            switch (my_heading) {
              case North: //NORTH
                cout << "North" << endl;
                //insert new id to path relative to last id based on heading
                temp_path.push_back(prev_cell_id-9);
                //get row/cols of potential location id (the following could be function
                //as it repeats for all switch cases)
                r = get_rc(temp_path.back(), 'r');
                c = get_rc(temp_path.back(), 'c');
                //compare location with map
                if(compare_walls(walls, r, c, map_walls) == true) {
                   //walls match so push to paths
                  *it = temp_path;
                  ++it;
                } else {
                  //check if current path is about to be destroyed
                  if (temp_path[0] == current_path_id) {
                    current_path_destroyed = true;
                  }
                  for (auto& e : *it) {
                    cout << e << endl;
                  }
                  if ((it = paths.erase(it)) == paths.end()) {
                    for (auto& e : *it) {
                    cout << e << endl;
                  }
                    break;
                  } 
                  for (auto& e : *it) {
                    cout << e << endl;
                  } 
                }
                break;
              case East: //EAST
                cout << "East" << endl;
                temp_path.push_back(prev_cell_id+1);
                r = get_rc(temp_path.back(), 'r');
                c = get_rc(temp_path.back(), 'c');
                if(compare_walls(walls, r, c, map_walls) == true) {
                  *it = temp_path;
                  ++it;
                } else {
                  if (temp_path[0] == current_path_id) {
                    current_path_destroyed = true;
                  }
                  for (auto& e : *it) {
                    cout << e << endl;
                  }
                  if ((it = paths.erase(it)) == paths.end()) {
                    for (auto& e : *it) {
                    cout << e << endl;
                  }
                    break;
                  } 
                  for (auto& e : *it) {
                    cout << e << endl;
                  }
                  //it = paths.begin();
                  
                }
                for (auto& e : paths) {
                  cout << "[";
                  for (auto& f : e) {
                    std::cout << f << ' ';
                  }
                  cout << "]" << endl;
                }
                break;
              case South: //SOUTH
                cout << "South" << endl;
                temp_path.push_back(prev_cell_id+9);
                r = get_rc(temp_path.back(), 'r');
                c = get_rc(temp_path.back(), 'c');
                if(compare_walls(walls, r, c, map_walls) == true) {
                  *it = temp_path;
                  ++it;
                } else {
                  if (temp_path[0] == current_path_id) {
                    current_path_destroyed = true;
                  }
                  for (auto& e : *it) {
                    cout << e << endl;
                  }
                  if ((it = paths.erase(it)) == paths.end()) {
                    for (auto& e : *it) {
                    cout << e << endl;
                  }
                    break;
                  } 
                  for (auto& e : *it) {
                    cout << e << endl;
                  }
                }
                break;
              case West: //WEST
                cout << "West" << endl;
                temp_path.push_back(prev_cell_id-1);
                r = get_rc(temp_path.back(), 'r');
                c = get_rc(temp_path.back(), 'c');
                if(compare_walls(walls, r, c, map_walls) == true) {
                  *it = temp_path;
                  ++it;
                } else {
                  if (temp_path[0] == current_path_id) {
                    current_path_destroyed = true;
                  }
                  for (auto& e : *it) {
                    cout << e << endl;
                  }
                  if ((it = paths.erase(it)) == paths.end()) {
                    for (auto& e : *it) {
                    cout << e << endl;
                  }
                    break;
                  } 
                  for (auto& e : *it) {
                    cout << e << endl;
                  }
                }
                break;
            }
          }
        } else if (*next_step == 'L') {
        // a turn has occured. adjust heading (could be function)
          switch (my_heading){
            case (North):
              my_heading = West;
              break;
            case (East):
              my_heading = North;
              break;
            case (South):
              my_heading = East;
              break;
            case (West):
              my_heading = South;
              break;
          }
        } else if (*next_step == 'R') {
          // a turn has occured. adjust heading.
          switch (my_heading){
            case (North):
              my_heading = East;
              break;
            case (East):
              my_heading = South;
              break;
            case (South):
              my_heading = West;
              break;
            case (West):
              my_heading = North;
              break;
          }
        }
        //instruction finished. Remove from instruction string.
        robot_instruct.erase(next_step);
        //Check if current path instructions are still valid
        if (current_path_destroyed == true) {
        
          smallestflood = 45;
          current_path_id = 0;
          // grab new shortest path from END of a NON GOAL ending path -> GOAL
          //find next smallest potential floodfill from current potential positions
          for (vector<vector<int>>::iterator it = paths.begin(); it != paths.end(); it++) {
            vector<int> temp_path = *it;
            int origin_r = get_rc(temp_path.front(), 'r');
            int origin_c = get_rc(temp_path.front(), 'c');
            int r = get_rc(temp_path.back(), 'r');
            int c = get_rc(temp_path.back(), 'c');
            
            if (origin_r != GOAL_ROW || origin_c != GOAL_COL) {
              if (floodfill_matrix[r][c] < smallestflood) {
                smallestflood = floodfill_matrix[r][c];
                current_path_id = temp_path.back();
              }
            }
          }
          get_short_path_from_id(floodfill_matrix, current_path_id, get_id(GOAL_ROW, GOAL_COL), current_path, walls); 
          for (auto& e : current_path) {
            cout << e << ' ';
            // path in e
          }
          cout << endl;
          robot_instruct = get_path_instruct(current_path_id, get_id(GOAL_ROW,GOAL_COL), my_heading, current_path, walls.hwalls, walls.vwalls);
         
          current_path_destroyed = false;
          cout << "Current Path ID: " << current_path_id << endl;
          cout << "New Instructions: " << robot_instruct << endl;
        }
        // print paths
          for (auto& e : paths) {
            cout << "[";
            for (auto& f : e) {
                std::cout << f << ' ';
            }
            cout << "]" << endl;
          }
      } else {
          smallestflood = 45;
          current_path_id = 0;
          // grab new shortest path from END of a NON GOAL ending path -> GOAL
          //find next smallest potential floodfill from current potential positions
          for (vector<vector<int>>::iterator it = paths.begin(); it != paths.end(); it++) {
            vector<int> temp_path = *it;
            int origin_r = get_rc(temp_path.back(), 'r');
            int origin_c = get_rc(temp_path.back(), 'c');
            int r = get_rc(temp_path.back(), 'r');
            int c = get_rc(temp_path.back(), 'c');
            
            if (origin_r != GOAL_ROW || origin_c != GOAL_COL) {
              if (floodfill_matrix[r][c] < smallestflood) {
                smallestflood = floodfill_matrix[r][c];
                current_path_id = temp_path.back();
              }
            }
          }
          get_short_path_from_id(floodfill_matrix, current_path_id, get_id(GOAL_ROW, GOAL_COL), current_path, walls); 
          for (auto& e : current_path) {
            cout << e << ' ';
            // path in e
          }
          cout << endl;
          robot_instruct = get_path_instruct(current_path_id, get_id(GOAL_ROW,GOAL_COL), my_heading, current_path, walls.hwalls, walls.vwalls);
          cout << "Current Path ID: " << current_path_id << endl;
          cout << "New Instructions: " << robot_instruct << endl;

      }
    } else {
      found_state = true;
      draw_potentials(walls, display, paths, found_state);
      cout << "Robot Localised!!" << endl;
      cout << "ROBOT INITIAL LOCATION WAS : [" << get_rc(paths[0][0], 'r') << ", " << get_rc(paths[0][0], 'c') << "]" << endl;
      cout << "heading = " << my_heading << endl;
      int robot_local_id = paths[0].back();
      vector<int> goal_path {};
      cout << "robot is now at " << robot_local_id << endl;
      get_short_path_from_id(floodfill_matrix, robot_local_id, get_id(GOAL_ROW, GOAL_COL), goal_path, walls);
      for (auto& e : goal_path) {
        cout << e << " ";
      }
      cout << endl;
      //goal_path.insert(goal_path.begin(), robot_local_id);
      string goal_instruct = get_path_instruct(robot_local_id, get_id(GOAL_ROW, GOAL_COL), my_heading, goal_path, walls.hwalls, walls.vwalls);
      cout << goal_instruct << endl;
      for (auto& e : goal_instruct) {
        robot->step(TIME_STEP);
        robot_follow_steps(e, robot, sensors, leftMotor, rightMotor, left_en, right_en);
      }
      cout << "Goal reached!!!" << endl;
      break;
    }
    draw_potentials(walls, display, paths, found_state);
  }
  
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
    } else { walls.hwalls[walls.row][walls.col] = 1;
    }
    if(sensors.right_ds->getValue() < 0.85){
      walls.vwalls[walls.row][walls.col+1] = 0;
    } else { walls.vwalls[walls.row][walls.col+1] = 1;
    }
    if(sensors.back_ds->getValue() < 0.85){
      walls.hwalls[walls.row+1][walls.col] = 0;
    }else { walls.hwalls[walls.row+1][walls.col] = 1;
    }
    if(sensors.left_ds->getValue() < 0.85){
      walls.vwalls[walls.row][walls.col] = 0;
    }else { walls.vwalls[walls.row][walls.col] = 1;
    }
  }   
  if (walls.heading_ == East){ //EAST
    if(sensors.front_ds->getValue() < 0.85){
      walls.vwalls[walls.row][walls.col+1] = 0;
    } else  {walls.vwalls[walls.row][walls.col+1] = 1;
    }
    if(sensors.right_ds->getValue() < 0.85){
      walls.hwalls[walls.row+1][walls.col] = 0;
    } else {walls.hwalls[walls.row+1][walls.col] = 1;
    }
    if(sensors.back_ds->getValue() < 0.85){
      walls.vwalls[walls.row][walls.col] = 0;
    } else {walls.vwalls[walls.row][walls.col] = 1;
    }
    if(sensors.left_ds->getValue() < 0.85){
      walls.hwalls[walls.row][walls.col] = 0; 
    } else {walls.hwalls[walls.row][walls.col] = 1;
    }
  }   
  if (walls.heading_ == South){ //SOUTH
    if(sensors.front_ds->getValue() < 0.85){
      walls.hwalls[walls.row+1][walls.col] = 0;
    } else { walls.hwalls[walls.row+1][walls.col] = 1;
    }
    if(sensors.right_ds->getValue() < 0.85){
      walls.vwalls[walls.row][walls.col] = 0;
    }else { walls.vwalls[walls.row][walls.col] = 1;
    }
    if(sensors.back_ds->getValue() < 0.85){
      walls.hwalls[walls.row][walls.col] = 0;
    }else { walls.hwalls[walls.row][walls.col] = 1;
    }
    if(sensors.left_ds->getValue() < 0.85){
      walls.vwalls[walls.row][walls.col+1] = 0;      
    }else { walls.vwalls[walls.row][walls.col+1] = 1;
    }
  }
  if (walls.heading_ == West){ //WEST    
    if(sensors.front_ds->getValue() < 0.85){
      walls.vwalls[walls.row][walls.col] = 0;
    } else {walls.vwalls[walls.row][walls.col] = 1;
    }
    if(sensors.right_ds->getValue() < 0.85){
      walls.hwalls[walls.row][walls.col] = 0;
    }else {walls.hwalls[walls.row][walls.col] = 1;
    }
    if(sensors.back_ds->getValue() < 0.85){
      walls.vwalls[walls.row][walls.col+1] = 0;
    }else {walls.vwalls[walls.row][walls.col+1] = 1;
    }
    if(sensors.left_ds->getValue() < 0.85){
      walls.hwalls[walls.row+1][walls.col] = 0;     
    }else {walls.hwalls[walls.row+1][walls.col] = 1;  
    }
  } 
}

void draw_map(walldata walls, Display *display) {

  //clear display
  display->setAlpha(0);
  display->fillRectangle(0,0,500,300);


 
//insert walls
  display->setColor(0xba25b0);
  //draw horizontal walls
  for(int x = 0; x < MAP_COLS; x++){
    for(int y = 1; y < MAP_ROWS; y++){
      if(walls.hwalls[y][x] == 0){
        display->setAlpha(0);
      }else if(walls.hwalls[y][x] == 1){
        display->setAlpha(1);
      }else {
        display->setAlpha(0.5);
      }
      int xcoord = DISP_COR_OFFSET + (x*DISP_CELL_STEP);
      int ycoord = DISP_COR_OFFSET + (y*DISP_CELL_STEP);
      display->drawLine(xcoord, ycoord, xcoord+DISP_CELL_STEP, ycoord);
    }
  } 
  //draw vertical walls
  for(int x = 1; x < MAP_COLS; x++){
    for(int y = 0; y < MAP_ROWS; y++){
      if(walls.vwalls[y][x] == 0){
        display->setAlpha(0);
      }else if(walls.vwalls[y][x] == 1){
        display->setAlpha(1);
      }else {
        display->setAlpha(0.5);
      }
      int xcoord = DISP_COR_OFFSET + (x*DISP_CELL_STEP);
      int ycoord = DISP_COR_OFFSET + (y*DISP_CELL_STEP);
      display->drawLine(xcoord, ycoord, xcoord, ycoord+DISP_CELL_STEP);
    }
  } 
  
  int rowpos = DISP_COR_OFFSET + (walls.row * DISP_CELL_STEP);
  int colpos = DISP_COR_OFFSET + (walls.col * DISP_CELL_STEP)+15;
  display->setAlpha(1);
  display->setFont("Arial", 30 , 1);
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

void draw_progress(walldata walls, Display *display){
  int Vsum = 0;
  int Hsum = 0;
  for(int x = 1; x < MAP_COLS; x++){
    for(int y = 0; y < MAP_ROWS; y++){
      if(walls.vwalls[y][x] != 2){
        Vsum++;
      }
    }
  }
  for(int x = 0; x < MAP_COLS; x++){
    for(int y = 1; y < MAP_ROWS; y++){
      if(walls.hwalls[y][x] != 2){
        Hsum++;
      }
    }
  }
  
  //display->setAlpha(0);
  //display->fillRectangle(70,240,405,265);
  
  float fraction = ((Vsum+Hsum)/76.0);
  cout<<fraction<<endl;
  display->setAlpha(1);
  display->setColor(0xbd0000);
  display->drawRectangle(105, 290, 295, 25);
  int progress = (295*fraction);
  display->fillRectangle(105, 290, progress, 25);
  display->setFont("Arial", 10 , 1);
  display->setColor(0xFFFFFF);
  display->drawText("PROGRESS", 212,297);
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

void check_walls_loc(Robot *robot, robotsensors sensors, walls_detected (&robot_walls)) {
  robot->step(TIME_STEP);
  robot_walls.N = true;
  robot_walls.E = true;
  robot_walls.S = true;
  robot_walls.W = true;
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
  if (!temp_path.empty()){
    temp_path.clear();
  }
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

string get_path_instruct(int start_id, int goal_id, heading init_heading, vector<int> path, int hWall[MAP_ROWS+1][MAP_COLS], int vWall[MAP_ROWS][MAP_COLS+1]) {
    // given a set of the ids of a path and robot's initial heading
    // determine the amount of turns required and generate turns
    int cur_id = start_id;
    string pathplan;
    //path.erase(cur_id);
    heading direction = North;
    heading my_heading = init_heading;
    while (cur_id != goal_id) {
        int r = get_rc(cur_id, 'r');
        int c = get_rc(cur_id, 'c');
        direction = North;
        // cout << "[" << path.front() << "]";
        for (int d = 0; d < 4; ++d) {
          //for all directions
            
            if (direction == North && r != 0 && hWall[r][c] == 0) {
                // std::cout << "check north" << std::endl;
                if (path.front() == get_id(r-1, c)) {
                    // next one is north
                    // set the cur_id to next nodes id
                    // cout << "next step is north" << endl;
                    cur_id = get_id(r-1, c);
                    path.erase(path.begin());
                    int a = dir_dif(my_heading, direction);
                    if (abs(a) == 2) {
                        // we are turning twice
                        pathplan += "LLF";
                    } else if (a == 1) {
                        // we are turning left
                        pathplan += "LF";
                    } else if (a == -1) {
                        // we are turning right
                        pathplan += "RF";
                    } else if (a == 0) {
                        // no turn just go forward
                        pathplan += "F";
                    }
                    // std::cout << "heading: " << my_heading << std::endl;
                    // cout << "dir_dif = " << a << endl;
                    my_heading = dir_trans(my_heading, a);
                    
                }
            } else if (direction == East && c != MAP_COLS - 1 && vWall[r][c+1] == 0) {
                // std::cout << "check east" << std::endl;
                if (path.front() == get_id(r, c+1)) {
                    // cout << "next step is east" << endl;
                    cur_id = get_id(r, c+1);
                    path.erase(path.begin());
                    int a = dir_dif(my_heading, direction);
                    if (abs(a) == 2) {
                        // we are turning twice
                        pathplan += "LLF";
                    } else if (a == 1) {
                        // we are turning left
                        pathplan += "LF";
                    } else if (a == -1) {
                        // we are turning right
                        pathplan += "RF";
                    } else if (a == 0) {
                        // no turn just go forward
                        pathplan += "F";
                    }
                    // std::cout << "heading: " << my_heading << std::endl;
                    // cout << "dir_dif = " << a << endl;
                    my_heading = dir_trans(my_heading, a);
                    
                }
            } else if (direction == South && r != MAP_ROWS - 1 && hWall[r+1][c] == 0) {
                // std::cout << "check south" << std::endl;
                if (path.front() == get_id(r+1, c)) {
                    // cout << "next step is south" << endl;
                    cur_id = get_id(r+1, c);
                    path.erase(path.begin());
                    int a = dir_dif(my_heading, direction);
                    if (abs(a) == 2) {
                        // we are turning twice
                        pathplan += "LLF";
                    } else if (a == 1) {
                        // we are turning left
                        pathplan += "LF";
                    } else if (a == -1) {
                        // we are turning right
                        pathplan += "RF";
                    } else if (a == 0) {
                        // no turn just go forward
                        pathplan += "F";
                    }
                    // std::cout << "heading: " << my_heading << std::endl;
                    // cout << "dir_dif = " << a << endl;
                    my_heading = dir_trans(my_heading, a);

                }
            } else if (direction == West && c != 0 && vWall[r][c] == 0) {
                // std::cout << "check west" << std::endl;
                if (path.front() == get_id(r, c-1)) {
                    // cout << "next step is west" << endl;
                    cur_id = get_id(r, c-1);
                    path.erase(path.begin());
                    int a = dir_dif(my_heading, direction);
                    if (abs(a) == 2) {
                        // we are turning twice
                        pathplan += "LLF";
                    } else if (a == 1) {
                        // we are turning left
                        pathplan += "LF";
                    } else if (a == -1) {
                        // we are turning right
                        pathplan += "RF";
                    } else if (a == 0) {
                        // no turn just go forward
                        pathplan += "F";
                    }
                    // std::cout << "heading: " << my_heading << std::endl;
                    // cout << "dir_dif = " << a << endl;
                    my_heading = dir_trans(my_heading, a);
                   
                }
            }
            advance_direction(direction);
        }
    }
    return(pathplan);
}

int dir_dif(heading a,heading b) {
    /*
    assigned direciton numbers are as such
         0
      3     1
         2
    thus, a difference of 1 is a single turn and a difference of 2 is 2 turns LL or RR
    */
    int dif = a - b;
    if (dif == -3) {
        dif = 1;
    } else if (dif == 3) {
        dif = -1;
    }
    return dif;
}

heading dir_trans(heading dir_num, int dir_dif) {
    int new_dir = dir_num - dir_dif;
    if (new_dir == 4) {
        new_dir = North;
    } else if (new_dir == -1) {
        new_dir = West;
    }
    return static_cast<heading>(new_dir);
}

// moves robot by following the instruct char
void robot_follow_steps(char instruct, Robot* robot, robotsensors (&sensors), 
                        Motor* leftMotor, Motor* rightMotor, 
                        PositionSensor *left_en, PositionSensor *right_en) {
  
  robot->step(TIME_STEP);
  double l_position = left_en->getValue();
  double r_position = right_en->getValue();
  
  cout << "instruction = " << instruct << endl;
  if (instruct == 'F') {
    //move forward
    leftMotor->setVelocity(0.8 * MAX_SPEED);
    rightMotor->setVelocity(0.8 * MAX_SPEED);
    double left_target_pos = l_position + TILE_STEP;
    double right_target_pos = r_position + TILE_STEP;
    leftMotor->setPosition(left_target_pos);
    rightMotor->setPosition(right_target_pos);
  
    while (left_target_pos-TOLERANCE >= left_en->getValue() && right_target_pos-TOLERANCE >= right_en->getValue()) {
      robot->step(TIME_STEP);
    }

  } else if (instruct == 'L') {
    //rotate left
    double left_target_pos = l_position - ROTATE_STEP;
    double right_target_pos = r_position + ROTATE_STEP;
    leftMotor->setVelocity(0.4 * MAX_SPEED);
    rightMotor->setVelocity(0.4 * MAX_SPEED);
    leftMotor->setPosition(left_target_pos);
    rightMotor->setPosition(right_target_pos);
    //read sesors to check position 
    while (left_target_pos+TOLERANCE <= left_en->getValue() && right_target_pos-TOLERANCE >= right_en->getValue()) {
      robot->step(TIME_STEP);
    }

    
  } else if (instruct == 'R') {
    //rotate right
    double left_target_pos = l_position + ROTATE_STEP;
    double right_target_pos = r_position - ROTATE_STEP;
    leftMotor->setVelocity(0.4 * MAX_SPEED);
    rightMotor->setVelocity(0.4 * MAX_SPEED);
    leftMotor->setPosition(left_target_pos);
    rightMotor->setPosition(right_target_pos);

    //read sesors to check position 
    while (left_target_pos-TOLERANCE >= left_en->getValue() && right_target_pos+TOLERANCE <= right_en->getValue()) {
      robot->step(TIME_STEP);
    }
  }
}

void draw_potentials(walldata walls, Display *display, vector<vector<int>> potentials, bool found_state) {

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
  display->setColor(0x0baaff);
  display->setFont("Arial", 30 , 1);
  if (found_state == true) {
    int rowpos = DISP_COR_OFFSET + (get_rc(potentials[0][0], 'r') * DISP_CELL_STEP);
    int colpos = DISP_COR_OFFSET + (get_rc(potentials[0][0], 'c') * DISP_CELL_STEP)+15;
    display->drawText("O", colpos, rowpos+8);
  } else {
    for (auto& e : potentials) {
      for (auto& f : e) {
        int rowpos = DISP_COR_OFFSET + (get_rc(f, 'r') * DISP_CELL_STEP);
        int colpos = DISP_COR_OFFSET + (get_rc(f, 'c') * DISP_CELL_STEP)+15;
        display->drawText("O", colpos, rowpos+8);
        }
      }
    }
    
    display->setColor(0x1f9e1f);
    display->drawRectangle(DISP_COR_OFFSET,DISP_COR_OFFSET,DISP_WIDTH+2,DISP_HEIGHT+2);
  }

  void scan_wall_values(walldata (&walls), std::vector<std::string> map) {
    
    int i = 0;
    // grab the sizes of walls in the map
    std::string hor_wall = HORIZONTAL_WALL;
    std::string ver_wall = VERTICAL_WALL;
    int hor_wall_size = hor_wall.length()/sizeof(char);
    int ver_wall_size = ver_wall.length()/sizeof(char);
    
    for (auto x = map.begin(); x != map.end(); ++x) {
        if (i % 2 == 0) {
            // scan for hWall
            for (size_t j = 0; j < 4*MAP_COLS; j += 4) {
                  //std::cout << "j is " << j << std::endl;
                  //std::cout << "string is " << *x << std::endl;
                  auto check = (*x).find(HORIZONTAL_WALL, j, hor_wall_size);
                  //std::cout << "check is " << check << std::endl;
                  if (check != std::string::npos && check == j+1) {
                      // wall present
                      walls.hwalls[i/2][j/4] = 1;
                  } else {
                      // wall not present
                      walls.hwalls[i/2][j/4] = 0;
                  }
            } 
        } else {
            // scan for vWall
            for (size_t j = 0; j < 4*MAP_COLS + 1; j += 4) {
                  //std::cout << "j is " << j << std::endl;
                  //std::cout << "string is " << *x << std::endl;
                  auto check = (*x).find(VERTICAL_WALL, j, ver_wall_size);
                  //std::cout << "check is " << check << std::endl;
                  if (check != std::string::npos && check == j) {
                      // wall present
                      walls.vwalls[i/2][j/4] = 1;
                  } else {
                      // wall not present
                      walls.vwalls[i/2][j/4] = 0;
                  }
             }
             
        }
        
        ++i;
        
    }
}