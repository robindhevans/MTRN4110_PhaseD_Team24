// File:          z5204873_MTRN4110_PhaseB.cpp
// Date:          1/07/2020
// Description:   Assignment Phase B
// Author:        Dean So (z5204873)
// Modifications:

// headers
#include <webots/Robot.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <queue>
#include <cstdlib>
#include <cmath>
#include <typeinfo>
#include <set>
#include <memory>
#include <stack>
#include <unordered_map>
#include <algorithm>
// definitions
#define MAP_FILE_NAME "../../Map.txt"
#define PATH_PLAN_FILE_NAME "../../PathPlanFound.txt"
#define PATH_PLAN_FOUND_FILE_NAME "./PathPlanFound.txt"
#define ANSWER_FILE_NAME "../../answer.txt"
#define MAZE_ROWS 5
#define MAZE_COLS 9
#define GOAL_ROW MAZE_ROWS/2
#define GOAL_COL MAZE_COLS/2
#define MAX_FLOOD ((MAZE_ROWS * MAZE_COLS) + 1)
#define HORIZONTAL_WALL "---"
#define VERTICAL_WALL "|"
// All the webots classes are defined in the "webots" namespace
using namespace webots;

// function prototypes
void scan_wall_values(int (&hWall)[MAZE_ROWS+1][MAZE_COLS], int (&vWall)[MAZE_ROWS][MAZE_COLS+1], std::vector<std::string> map);
void scan_robot_pos(int (&robot_pos)[2], char &heading, std::vector<std::string> map);
void advance_direction(char &direction);
void print_flood_maze(int path[MAZE_ROWS][MAZE_COLS], std::vector<std::string> display_map, int robot_pos[2]);
void get_path_set_flood(std::set<int> visited, int (&path)[MAZE_ROWS][MAZE_COLS], int cells[MAZE_ROWS][MAZE_COLS]);
int get_rc(int id, char setting);
int get_id(int r, int c);
void dfs_shortPaths(int start_id, int goal_id, std::unordered_map<int, std::set<int>> (&nodeParents), std::vector<std::set<int>> (&shortPaths), std::set<int> path);
int path_turns(int start_id, int goal_id, char init_heading, std::set<int> path, int (&hWall)[MAZE_ROWS+1][MAZE_COLS], int (&vWall)[MAZE_ROWS][MAZE_COLS+1]);
void path_plan(int start_id, int goal_id, char init_heading, std::set<int> path, int (&hWall)[MAZE_ROWS+1][MAZE_COLS], int (&vWall)[MAZE_ROWS][MAZE_COLS+1], std::string &pathplan);
char dir_card(int dir_num);
int dir_num(char direction);
int dir_dif(int a, int b);
char dir_trans(int dir_num, int dir_dif);

// main function
int main(int argc, char **argv) {
    Robot *robot = new Robot();
    // get the time step of the current world.
    int timeStep = (int)robot->getBasicTimeStep();
    
    // direct all standard output to answer.txt
    // make sure to comment out the line at end of code that resets the std ouput
    /*
    std::ofstream out(ANSWER_FILE_NAME);
    std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
    std::cout.rdbuf(out.rdbuf()); //redirect std::cout to out.txt!
    */
    
    
    // read map.txt
    //std::cout << "Start! Reading Map from \"" << MAP_FILE_NAME << "\"" << std::endl;
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
    
    std::cout << "--- Task 1 ---" << std::endl;
    for (auto x = map.begin(); x != map.end(); ++x) {
        std::cout << *x << std::endl;
    }
    
    //std::cout << "Done - Map read!" << std::endl;
    
    // interpret the map into a data structure
    // store separate arrays for horizontal and vertical walls as well as cell values
    // initialise the arrays
    int robot_pos[2] = {}; // position defined from 0 in row and column rather than 1
    char heading = 0;
    scan_robot_pos(robot_pos, heading, map);
    //std::cout << "initial pos = [" << robot_pos[0] << "," << robot_pos[1] << "]" << std::endl;
    int hWall[MAZE_ROWS+1][MAZE_COLS] = {};
    int vWall[MAZE_ROWS][MAZE_COLS+1] = {};
    scan_wall_values(hWall, vWall, map);
    /*
    std::cout << "HORIZONTAL WALL MATRIX: " << std::endl;
    for (const auto& e : hWall) {
        for (const auto& f : e) {
            std::cout << f << " ";
        }
        std::cout << std::endl;
    }
    
    std::cout << "VERTICAL WALL MATRIX: " << std::endl;
    for (const auto& e : vWall) {
        for (const auto& f : e) {
            std::cout << f << " ";
        }
        std::cout << std::endl;
    }
    */
    // apply flood fill algorithm
    // initialise cells to N
    int N = MAX_FLOOD;
    int cells[MAZE_ROWS][MAZE_COLS] = {};
    for (auto& e : cells) {
        for (auto& f : e) {
            f = N;
        }
    }
    // set goal cell to 0
    cells[GOAL_ROW][GOAL_COL] = 0;
    /*
    // print cells
    std::cout << "CELL VALUES: " << std::endl;
    for (const auto& e : cells) {
        for (const auto& f : e) {
            std::cout << f << " ";
        }
        std::cout << std::endl;
    }
    */
    int cur_explored_cell = 0;
    bool maze_value_changed = true;
    char direction = 'N';
    //int dir_mod = 0;
    // flood fill loop
    while (maze_value_changed != false) {
        maze_value_changed = false;
        for (int r = 0; r < MAZE_ROWS; ++r) {
            for (int c = 0; c < MAZE_COLS; ++c) {
                if (cells[r][c] == cur_explored_cell) {
                    direction = 'N';
                    //dir_mod = 0;
                    for (int d = 0; d < 4; ++d) {
                        if (direction == 'N' && r != 0 && hWall[r][c] == 0) {
                            // if searching north but at top row dont need to update anything
                            if (cells[r-1][c] == N) {
                                cells[r-1][c] = cells[r][c] + 1;
                                maze_value_changed = true;
                            }

                        } else if (direction == 'E' && c != MAZE_COLS - 1 && vWall[r][c+1] == 0) {
                            // if searching east but at right most col dont need to update anything
                            if (cells[r][c+1] == N) {
                                cells[r][c+1] = cells[r][c] + 1;
                                maze_value_changed = true;
                            }
                        } else if (direction == 'S' && r != MAZE_ROWS - 1 && hWall[r+1][c] == 0) {
                            // if searching south but at bottom row dont need to update anything
                            if (cells[r+1][c] == N) {
                                cells[r+1][c] = cells[r][c] + 1;
                                maze_value_changed = true;
                            }
                        } else if (direction == 'W' && c != 0 && vWall[r][c] == 0) {
                            // if searching west but at left most col dont need to update anything
                            if (cells[r][c-1] == N) {
                                cells[r][c-1] = cells[r][c] + 1;
                                maze_value_changed = true;
                            }
                        }
                        advance_direction(direction);
                    }
                }
            }
        }
        ++cur_explored_cell;
    }
    // print flood filled maze
    // each path is stored in a vector of short paths
    
    // std::vector<std::string> display_map;
    // copy(map.begin(), map.end(), back_inserter(display_map));
    // print_flood_maze(cells, display_map, robot_pos);
    // find shortest path
    
    // construct tree of shortest paths
    std::queue<int> pathQ;
    // visited is defined by a node id
    // each cell is assigned a certain id
    std::set<int> visited;
    
    std::unordered_map<int, std::set<int>> nodeParents;
    int robot_cur = get_id(robot_pos[0], robot_pos[1]);
    pathQ.push(robot_cur);
    visited.emplace(robot_cur);
    
    while(!pathQ.empty()) {
        int cur_id = pathQ.front();
        int r = get_rc(pathQ.front(), 'r');
        int c = get_rc(pathQ.front(), 'c');
        pathQ.pop();
        visited.emplace(cur_id);
        //std::cout << "current cell/node: " << "[" << r << "," << c << "]";
        if (r == GOAL_ROW && c == GOAL_COL) {
            // goal node found
            
            //std::cout << std::endl;
            break;
        }
        if (nodeParents.find(cur_id) == nodeParents.end()) {
            nodeParents.emplace(cur_id, std::set<int>{});
        }
        // for all neighbouring nodes = for all directions/neighbouring cells
        direction = 'N';
        for (int d = 0; d < 4; ++d) {
            // only traverse as a bfs node if valid node in that direction
            // - no wall in direction
            // - cell value = current cell value - 1
            if (direction == 'N' && r != 0 && hWall[r][c] == 0) {
                //std::cout << "check north" << std::endl;
                if (cells[r-1][c] == cells[r][c] - 1) {
                    if ((visited.find(get_id(r-1, c))) == visited.end()) {
                        pathQ.push(get_id(r-1, c));
                        
                        if (nodeParents.find(get_id(r-1, c)) == nodeParents.end()) {
                            nodeParents.emplace(get_id(r-1, c), std::set<int>{});
                        }
                        auto temp = nodeParents.find(get_id(r-1, c));
                        (*temp).second.emplace(cur_id);
                    }
                }
            } else if (direction == 'E' && c != MAZE_COLS - 1 && vWall[r][c+1] == 0) {
                //std::cout << "check east" << std::endl;
                if (cells[r][c+1] == cells[r][c] - 1) {
                    if ((visited.find(get_id(r, c+1))) == visited.end()) {
                        pathQ.push(get_id(r, c+1));
                        
                        if (nodeParents.find(get_id(r, c+1)) == nodeParents.end()) {
                            nodeParents.emplace(get_id(r, c+1), std::set<int>{});
                        }
                        auto temp = nodeParents.find(get_id(r, c+1));
                        (*temp).second.emplace(cur_id);
                    }
                }
            } else if (direction == 'S' && r != MAZE_ROWS - 1 && hWall[r+1][c] == 0) {
                //std::cout << "check south" << std::endl;
                if (cells[r+1][c] == cells[r][c] - 1) {
                    if ((visited.find(get_id(r+1, c))) == visited.end()) {
                        pathQ.push(get_id(r+1, c));
                        
                        if (nodeParents.find(get_id(r+1, c)) == nodeParents.end()) {
                            nodeParents.emplace(get_id(r+1, c), std::set<int>{});
                        }
                        auto temp = nodeParents.find(get_id(r+1, c));
                        (*temp).second.emplace(cur_id);
                    }
                }
            } else if (direction == 'W' && c != 0 && vWall[r][c] == 0) {
                //std::cout << "check west" << std::endl;
                if (cells[r][c-1] == cells[r][c] - 1) {
                    if ((visited.find(get_id(r, c-1))) == visited.end()) {
                        pathQ.push(get_id(r, c-1));
                        
                        if (nodeParents.find(get_id(r, c-1)) == nodeParents.end()) {
                            nodeParents.emplace(get_id(r, c-1), std::set<int>{});
                        }
                        auto temp = nodeParents.find(get_id(r, c-1));
                        (*temp).second.emplace(cur_id);
                    }
                }
            }
            advance_direction(direction);
        }
        //std::cout << std::endl;
    }
    /*
    for (const auto& e : nodeParents) {
        std::cout << "[" << get_rc(e.first, 'r') << ", " << get_rc(e.first, 'c') << "] " << "Parents = ";
        for (const auto& f : e.second) {
            std::cout << "{" << get_rc(f, 'r') << ", " << get_rc(f, 'c') << "} ";
        }
        std::cout << std::endl;
    }
    */
    /*
    for (const &e : visited) {
        std::cout << e << std::endl;
    }
    */
    std::vector<std::set<int>> shortPaths;  // vector of path structs which store each path
    std::set<int> path;
    int start_id = get_id(robot_pos[0], robot_pos[1]);
    int goal_id = get_id(GOAL_ROW, GOAL_COL);
    dfs_shortPaths(start_id, goal_id, nodeParents, shortPaths, path);
    /*
    for (const auto& e : shortPaths) {
        for (const auto& f : e) {
            std::cout << "{" << get_rc(f, 'r') << ", " << get_rc(f, 'c') << "} ";
        }
        std::cout << std::endl;
    }
    */
    std::vector<int> pathTurns;
    
    int i = 0;
    std::cout << "--- Task 2 ---" << std::endl;
    // print every shortest path and calculate the turns for each path
    for (auto&e : shortPaths) {
        std::cout << "--- Path " << i + 1 << " ---" << std::endl;
        int temp[MAZE_ROWS][MAZE_COLS] = {};
        get_path_set_flood(e, temp, cells);
        std::vector<std::string> display_map;
        copy(map.begin(), map.end(), back_inserter(display_map));
        print_flood_maze(temp, display_map, robot_pos);
        int turns = path_turns(start_id, goal_id, heading, e, hWall, vWall);
        pathTurns.push_back(turns);
        ++i;
    }
    
    std::cout << "--- Task 3 ---" << std::endl;
    auto it_shortTurns = min_element(pathTurns.begin(), pathTurns.end());
    auto index_shortTurns = it_shortTurns - pathTurns.begin();
    int tempx[MAZE_ROWS][MAZE_COLS] = {};
    get_path_set_flood(shortPaths[index_shortTurns], tempx, cells);
    std::vector<std::string> display_mapx;
    copy(map.begin(), map.end(), back_inserter(display_mapx));
    print_flood_maze(tempx, display_mapx, robot_pos);
    // construct the path plan 
    std::string pathplan = {};
    path_plan(start_id, goal_id, heading, shortPaths[index_shortTurns], hWall, vWall, pathplan);
    std::cout << "Steps: " << pathplan.length() << std::endl;
    char row = robot_pos[0] + '0';
    char col = robot_pos[1] + '0';
    pathplan.insert(pathplan.begin(), heading);
    pathplan.insert(pathplan.begin(), col);
    pathplan.insert(pathplan.begin(), row);
    std::cout << "Path: " << pathplan << std::endl;
    
    // output path plan to a file PathPlanFound.txt
    std::ofstream pathplanfound (PATH_PLAN_FILE_NAME);
    if (pathplanfound.is_open()) {
        pathplanfound << pathplan;
        pathplanfound.close();
    } else {
        std::cout << "Unable to open file" << std::endl;
    }
    
    std::cout << "--- Task 4 ---" << std::endl;
    std::cout << "File: " << PATH_PLAN_FILE_NAME << std::endl;
    std::cout << "Path: " << pathplan << std::endl;
    while (robot->step(timeStep) != -1) {
        break;
    }
    // resets std output to old buffer - stops redirecting to answer.txt
    /*
    std::cout.rdbuf(coutbuf);
    */
    delete robot;
    return 0;
}

void scan_wall_values(int (&hWall)[MAZE_ROWS+1][MAZE_COLS], int (&vWall)[MAZE_ROWS][MAZE_COLS+1], std::vector<std::string> map) {
    
    int i = 0;
    // grab the sizes of walls in the map
    std::string hor_wall = HORIZONTAL_WALL;
    std::string ver_wall = VERTICAL_WALL;
    int hor_wall_size = hor_wall.length()/sizeof(char);
    int ver_wall_size = ver_wall.length()/sizeof(char);
    
    for (auto x = map.begin(); x != map.end(); ++x) {
        if (i % 2 == 0) {
            // scan for hWall
            for (size_t j = 0; j < 4*MAZE_COLS; j += 4) {
                  //std::cout << "j is " << j << std::endl;
                  //std::cout << "string is " << *x << std::endl;
                  auto check = (*x).find(HORIZONTAL_WALL, j, hor_wall_size);
                  //std::cout << "check is " << check << std::endl;
                  if (check != std::string::npos && check == j+1) {
                      // wall present
                      hWall[i/2][j/4] = 1;
                  } else {
                      // wall not present
                      hWall[i/2][j/4] = 0;
                  }
            } 
        } else {
            // scan for vWall
            for (size_t j = 0; j < 4*MAZE_COLS + 1; j += 4) {
                  //std::cout << "j is " << j << std::endl;
                  //std::cout << "string is " << *x << std::endl;
                  auto check = (*x).find(VERTICAL_WALL, j, ver_wall_size);
                  //std::cout << "check is " << check << std::endl;
                  if (check != std::string::npos && check == j) {
                      // wall present
                      vWall[i/2][j/4] = 1;
                  } else {
                      // wall not present
                      vWall[i/2][j/4] = 0;
                  }
             }
             
        }
        
        ++i;
        
    }
}

void scan_robot_pos(int (&robot_pos)[2], char &heading, std::vector<std::string> map) {
    int i = 0;
    bool found_robot = false;
    for (auto x = map.begin(); x != map.end(); ++x) {
        if (i % 2 != 0) {
            // scan for robot position
            for (int j = 2; j < 4*MAZE_COLS; j += 4) {
                switch((*x)[j]) {
                    case 'v':
                        heading = 'S';
                        found_robot = true;
                        break;
                    case '<':
                        heading = 'W';
                        found_robot = true;
                        break;
                    case '>':
                        heading = 'E';
                        found_robot = true;
                        break;
                    case '^':
                        heading = 'N';
                        found_robot = true;
                        break;
                }
                if (found_robot == true) {
                    robot_pos[1] = (j-2)/4;
                    break;
                }
            }
            if (found_robot == true) {
                    robot_pos[0] = i/2;
                    break;
            }
        }
        ++i;
        
    }
}

void advance_direction(char &direction) {
    switch (direction) {
        case 'N':
            direction = 'E';
            //dir_mod = 1;
            break;
        case 'E':
            direction = 'S';
            //dir_mod = 1;
            break;
        case 'S':
            direction = 'W';
            //dir_mod = 0;
            break;
        case 'W':
            direction = 'N';
            //dir_mod = 0;
            break;
    }
}

void print_flood_maze(int path[MAZE_ROWS][MAZE_COLS], std::vector<std::string> display_map, int robot_pos[2]) {
    int i = 0;
    for (auto x = display_map.begin(); x != display_map.end(); ++x) {
        if (i % 2 != 0) {
            for (int j = 2; j < 4*MAZE_COLS; j += 4) {
                if (i/2 == robot_pos[0] && ((j-2)/4) == robot_pos[1]) {
                    continue;
                }
                int num = path[i/2][(j-2)/4];
                std::string temp = std::to_string(num);
                if (num >= 0) {
                    if (num >= 10) {
                        (*x)[j] = temp[0];
                        (*x)[j+1] = temp[1];
                    } else {
                        (*x)[j] = temp[0];
                    }
                }
            }    
        }
        ++i;
    }
    for (auto x = display_map.begin(); x != display_map.end(); ++x) {
        std::cout << *x << std::endl;
    }
}

void get_path_set_flood(std::set<int> path_set, int (&path)[MAZE_ROWS][MAZE_COLS], int cells[MAZE_ROWS][MAZE_COLS]) {
    for (int i = 0; i < MAZE_ROWS; ++i) {
        for (int j = 0; j < MAZE_COLS; ++j) {
            path[i][j] = -2;
        }
    }
    for (const &e : path_set) {
        int r = get_rc(e, 'r');
        int c = get_rc(e, 'c');
        path[r][c] = cells[r][c];
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

void dfs_shortPaths(int start_id, int goal_id, std::unordered_map<int, std::set<int>> (&nodeParents), std::vector<std::set<int>> (&shortPaths), std::set<int> path) {
    // traverse through the nodeParents and generate the paths by 
    // pushing each set of indexes of the path into a vector
    path.insert(path.begin(), goal_id);
    
    if (goal_id == start_id) {
        shortPaths.push_back(path);
    } else {
        if (nodeParents.find(goal_id) != nodeParents.end()) {
            std::set<int> parents = nodeParents.find(goal_id)->second;
            for (auto const& parent : parents) {
                dfs_shortPaths(start_id, parent, nodeParents, shortPaths, path);
            }
        }
    }
    path.erase(path.begin());
}

int path_turns(int start_id, int goal_id, char init_heading, std::set<int> path, int (&hWall)[MAZE_ROWS+1][MAZE_COLS], int (&vWall)[MAZE_ROWS][MAZE_COLS+1]) {
    // given a set of the ids of a path and robot's initial heading
    // determine the amount of turns required and generate turns
    int turns = 0;
    int cur_id = start_id;
    path.erase(cur_id);
    char direction = 'N';
    char heading = init_heading;
    while (cur_id != goal_id) {
        int r = get_rc(cur_id, 'r');
        int c = get_rc(cur_id, 'c');
        direction = 'N';
        
        for (int d = 0; d < 4; ++d) {
        
            if (direction == 'N' && r != 0 && hWall[r][c] == 0) {
                //std::cout << "check north" << std::endl;
                if ((path.find(get_id(r-1, c))) != path.end()) {
                    // next one is north
                    // set the cur_id to next nodes id
                    cur_id = get_id(r-1, c);
                    path.erase(cur_id);
                    int a = dir_dif(dir_num(heading), dir_num(direction));
                    if (a == 2) {
                        // we are turning twice
                        turns += 2;
                    } else if (a == 1) {
                        // we are turning left
                        turns += 1;
                    } else if (a == -1) {
                        // we are turning right
                        turns += 1;
                    } else if (a == 0) {
                        // no turn just go forward
                    }
                    heading = dir_trans(dir_num(heading), a);
                    //std::cout << "heading: " << heading << std::endl;
                    
                }
            } else if (direction == 'E' && c != MAZE_COLS - 1 && vWall[r][c+1] == 0) {
                //std::cout << "check east" << std::endl;
                if ((path.find(get_id(r, c+1))) != path.end()) {
                    cur_id = get_id(r, c+1);
                    path.erase(cur_id);
                    int a = dir_dif(dir_num(heading), dir_num(direction));
                    if (a == 2) {
                        // we are turning twice
                        turns += 2;
                    } else if (a == 1) {
                        // we are turning left
                        turns += 1;
                    } else if (a == -1) {
                        // we are turning right
                        turns += 1;
                    } else if (a == 0) {
                        // no turn just go forward
                    }
                    heading = dir_trans(dir_num(heading), a);
                    //std::cout << "heading: " << heading << std::endl;
                }
            } else if (direction == 'S' && r != MAZE_ROWS - 1 && hWall[r+1][c] == 0) {
                //std::cout << "check south" << std::endl;
                if ((path.find(get_id(r+1, c))) != path.end()) {
                    cur_id = get_id(r+1, c);
                    path.erase(cur_id);
                    int a = dir_dif(dir_num(heading), dir_num(direction));
                    if (a == 2) {
                        // we are turning twice
                        turns += 2;
                    } else if (a == 1) {
                        // we are turning left
                        turns += 1;
                    } else if (a == -1) {
                        // we are turning right
                        turns += 1;
                    } else if (a == 0) {
                        // no turn just go forward
                    }
                    heading = dir_trans(dir_num(heading), a);
                    //std::cout << "heading: " << heading << std::endl;
                }
            } else if (direction == 'W' && c != 0 && vWall[r][c] == 0) {
                //std::cout << "check west" << std::endl;
                if ((path.find(get_id(r, c-1))) != path.end()) {
                    cur_id = get_id(r, c-1);
                    path.erase(cur_id);
                    int a = dir_dif(dir_num(heading), dir_num(direction));
                    if (a == 2) {
                        // we are turning twice
                        turns += 2;
                    } else if (a == 1) {
                        // we are turning left
                        turns += 1;
                    } else if (a == -1) {
                        // we are turning right
                        turns += 1;
                    } else if (a == 0) {
                        // no turn just go forward
                    }
                    heading = dir_trans(dir_num(heading), a);
                    //std::cout << "heading: " << heading << std::endl;
                }
            }
            advance_direction(direction);
        }
    }
    return turns;
}

void path_plan(int start_id, int goal_id, char init_heading, std::set<int> path, int (&hWall)[MAZE_ROWS+1][MAZE_COLS], int (&vWall)[MAZE_ROWS][MAZE_COLS+1], std::string &pathplan) {
    // given a set of the ids of a path and robot's initial heading
    // determine the amount of turns required and generate turns
    int cur_id = start_id;
    path.erase(cur_id);
    char direction = 'N';
    char heading = init_heading;
    while (cur_id != goal_id) {
        int r = get_rc(cur_id, 'r');
        int c = get_rc(cur_id, 'c');
        direction = 'N';
        
        for (int d = 0; d < 4; ++d) {
        
            if (direction == 'N' && r != 0 && hWall[r][c] == 0) {
                //std::cout << "check north" << std::endl;
                if ((path.find(get_id(r-1, c))) != path.end()) {
                    // next one is north
                    // set the cur_id to next nodes id
                    cur_id = get_id(r-1, c);
                    path.erase(cur_id);
                    int a = dir_dif(dir_num(heading), dir_num(direction));
                    if (a == 2) {
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
                    heading = dir_trans(dir_num(heading), a);
                    //std::cout << "heading: " << heading << std::endl;
                    
                }
            } else if (direction == 'E' && c != MAZE_COLS - 1 && vWall[r][c+1] == 0) {
                //std::cout << "check east" << std::endl;
                if ((path.find(get_id(r, c+1))) != path.end()) {
                    cur_id = get_id(r, c+1);
                    path.erase(cur_id);
                    int a = dir_dif(dir_num(heading), dir_num(direction));
                    if (a == 2) {
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
                    heading = dir_trans(dir_num(heading), a);
                    //std::cout << "heading: " << heading << std::endl;
                }
            } else if (direction == 'S' && r != MAZE_ROWS - 1 && hWall[r+1][c] == 0) {
                //std::cout << "check south" << std::endl;
                if ((path.find(get_id(r+1, c))) != path.end()) {
                    cur_id = get_id(r+1, c);
                    path.erase(cur_id);
                    int a = dir_dif(dir_num(heading), dir_num(direction));
                    if (a == 2) {
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
                    heading = dir_trans(dir_num(heading), a);
                    //std::cout << "heading: " << heading << std::endl;
                }
            } else if (direction == 'W' && c != 0 && vWall[r][c] == 0) {
                //std::cout << "check west" << std::endl;
                if ((path.find(get_id(r, c-1))) != path.end()) {
                    cur_id = get_id(r, c-1);
                    path.erase(cur_id);
                    int a = dir_dif(dir_num(heading), dir_num(direction));
                    if (a == 2) {
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
                    heading = dir_trans(dir_num(heading), a);
                    //std::cout << "heading: " << heading << std::endl;
                }
            }
            advance_direction(direction);
        }
    }
}

char dir_card(int dir_num) {
    char card = 'N';
    if (dir_num == 0) {
        card = 'N';
    } else if (dir_num == 1) {
        card = 'E';
    } else if (dir_num == 2) {
        card = 'S';
    } else if (dir_num == 3) {
        card = 'W';
    }
    return card;
}

int dir_num(char direction) {
    int num = 0;
    if (direction == 'N') {
        num = 0;
    } else if (direction == 'E') {
        num = 1;
    } else if (direction == 'S') {
        num = 2;
    } else if (direction == 'W') {
        num = 3;
    }
    return num;
}

int dir_dif(int a, int b) {
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

char dir_trans(int dir_num, int dir_dif) {
    int new_dir = dir_num - dir_dif;
    if (new_dir == 4) {
        new_dir = 0;
    } else if (new_dir == -1) {
        new_dir = 3;
    }
    return dir_card(new_dir);
}