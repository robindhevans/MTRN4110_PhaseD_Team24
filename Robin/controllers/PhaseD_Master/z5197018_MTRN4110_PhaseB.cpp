// File:          z5197018_MTRN4110_PhaseB.cpp
// Date:
// Description: Path Planning program that reads a text file graphical representation of
              //a 9x5 maze and plots the shortest path with least turns, outputting instructions
              //to a separate text file. Path planning is via floodfill algorithm, combined
              //with a modified Depth First Search binary search tree method.
              //
              //NOTE: This program allows for a maximum of 2000 potential shortest paths, enough to cover an 
              //enclosed 9x5 maze. It therefore requires enough memory to hold a 2000x45 array of pointers to store the paths.
              //NOTE2: This program outputs start position coordinates as YX instead of traditional
              //cartesian XY due to preferred marking output using row/col indexing as coordinates.
// Author: Robin Evans

#include <webots/Robot.hpp>
#include <iostream>
#include <fstream>
#include <stack>
#include <list>
#include <cstdlib>
#include <cmath>

#define MAP_FILE_NAME "../../Map.txt"
#define PATH_PLAN_FILE_NAME "../../PathPlan.txt"
#define MAX_STEPS 45
#define MAX_PATHS 2000
#define TRUE 1
#define FALSE 0
#define GOAL_X 4
#define GOAL_Y 2

using namespace webots;
using namespace std;

enum Heading {North = 0, East, South, West};

struct node {
  int x, y, val;
  Heading dir_from_parent;
  struct node *left, *right;
  node() {
    x = 0;
    y = 0;
    val = -1;
    dir_from_parent = North;
    left = right = NULL;
  }
  node(int x_, int y_, int val_, Heading dir_from_parent_) {
    this->x = x_;
    this->y = y_;
    this->val = val_;
    this->dir_from_parent = dir_from_parent_;
    left = right = NULL;
  }
  
};

int z5197018_MTRN4110_PhaseB() {

  // define horizontal wall memory
  bool wall_hori [6][9] = {FALSE};
  // define vertical wall memory
  bool wall_vert [5][10] = {FALSE};
  // define cell value matrix
  int matrix [5][9];
  //cout << "Initial Flood Matrix" << endl;
  // initialise cell value matrix
  for(int i = 0; i < 5; i++) {
    for(int j = 0; j < 9; j++) {
      matrix[i][j] = 45;
      //cout << matrix[i][j] << ' ';
    }
    //cout << endl;
  }
  // initialise goal value as zero
  matrix[GOAL_Y][GOAL_X] = 0;
  
  //open map file
  cout << "--- Task 1 ---" << endl;
  ifstream maze_map;
  maze_map.open(MAP_FILE_NAME);
  if(maze_map.is_open()) {
    char line [40];
    while(maze_map.getline(line, 40)) {
      cout << line << endl;
    }
  }
  else {
    cerr << "Error: No map file to open. Terminating." << endl;
    return 1;
  }
  //cout << "Done - Map read!" << endl;
  
  maze_map.clear();
  maze_map.seekg(0, maze_map.beg);
  char c;
  bool wall_dir = FALSE;
  int wall_hori_i = 0;
  int wall_hori_j = 0;
  int wall_vert_i = 0;
  int wall_vert_j = 0;
  struct node* root = new node();
  // ------------Read map-----------//
  while(maze_map.get(c)) {
  // check if verti or hori row
    if (wall_dir == FALSE) {
  //if hori row:
      maze_map.get(c);
      switch(c) {
        // check for new line char
        case('\n'):        
          wall_dir = TRUE;
          wall_hori_i++;
          wall_hori_j = 0;
          //cout << "endline" << endl;
          break;
        //check for no wall
        case(' '):
          maze_map.get(c);
          maze_map.get(c);
          //cout << wall_hori[wall_hori_i][wall_hori_j] << endl;
          wall_hori_j++;
          break;
        //check for wall
        case('-'):
          wall_hori[wall_hori_i][wall_hori_j] = TRUE;
          maze_map.get(c);
          maze_map.get(c);
          //cout << wall_hori[wall_hori_i][wall_hori_j] << endl;
          wall_hori_j++;
          break;
      }
    }
    else {
  //if verti row:
      switch(c) {
        // check for new line
        case('\n'):
          wall_vert_i++;
          wall_dir = FALSE;
          wall_vert_j = 0;
          break;
        // check for no wall
        case(' '):
          maze_map.get(c);
          maze_map.get(c);
         
          // check for starting cell and direction
          if(c == '^') {root->x = wall_vert_j; root->y = wall_vert_i; root->dir_from_parent = North;}
          if(c == '>') {root->x = wall_vert_j; root->y = wall_vert_i; root->dir_from_parent = East;}
          if(c == 'v') {root->x = wall_vert_j; root->y = wall_vert_i; root->dir_from_parent = South;}
          if(c == '<') {root->x = wall_vert_j; root->y = wall_vert_i; root->dir_from_parent = West;}
          maze_map.get(c);
          wall_vert_j++;
          break;
        // check for wall
        case('|'):
          wall_vert[wall_vert_i][wall_vert_j] = TRUE;
          // check for starting cell and direction
          if(wall_vert_j < 9) {
            maze_map.get(c);
            maze_map.get(c);
            if(c == '^') {root->x = wall_vert_j; root->y = wall_vert_i; root->dir_from_parent = North;}
            if(c == '>') {root->x = wall_vert_j; root->y = wall_vert_i; root->dir_from_parent = East;}
            if(c == 'v') {root->x = wall_vert_j; root->y = wall_vert_i; root->dir_from_parent = South;}
            if(c == '<') {root->x = wall_vert_j; root->y = wall_vert_i; root->dir_from_parent = West;}
            maze_map.get(c);
          }
          wall_vert_j++;
      }
    }
  }
  //cout << root->x << root->y << ' ' << root->dir_from_parent << endl;
  // Debug Verti and Hori matrix
  /*
  cout << "Vertical Wall Matrix" << endl;
  for(int i = 0; i < 5; i++) {
    for(int j = 0; j < 10; j++) {
      cout << wall_vert[i][j];
    }
    cout << endl;
  }
  cout << "Horizontal Wall Matrix" << endl;
  for(int i = 0; i < 6; i++) {
    for(int j = 0; j < 9; j++) {
      cout << wall_hori[i][j];
    }
    cout << endl;
  }
  */
  //------------floodfill algorithm------------//
  // start exploration of matrix with goal
  cout << "--- Task 2 ---" << endl;
  int current_explored_val = 0;
  bool maze_val_changed = TRUE;
  while(maze_val_changed == TRUE) {
    maze_val_changed = FALSE;
    for(int i = 0; i < 5; i++) {
      for(int j = 0; j < 9; j++) {
        //check if current cell has curent value
        if (matrix[i][j] == current_explored_val) {
          //check north for wall and unexplored
          if (wall_hori[i][j] == FALSE) {
            if(matrix[i-1][j] == 45) {
              matrix[i-1][j] = current_explored_val + 1;
              maze_val_changed = TRUE;
            }
          }
          //check south for wall and unexplored
          if (wall_hori[i+1][j] == FALSE) {
            if(matrix[i+1][j] == 45) {
              matrix[i+1][j] = current_explored_val + 1;
              maze_val_changed = TRUE;
            }
          }
          //check west for wall and unexplored
          if (wall_vert[i][j] == FALSE) {
            if(matrix[i][j-1] == 45) {
              matrix[i][j-1] = current_explored_val + 1;
              maze_val_changed = TRUE;
            }
          }
          //check east for wall and unexplored
          if (wall_vert[i][j+1] == FALSE) {
            if(matrix[i][j+1] == 45) {
              matrix[i][j+1] = current_explored_val + 1;
              maze_val_changed = TRUE;
            }
          }
        }
      }
    }
  //increment explored value
  current_explored_val++;
  }
  // update origin/root with floodfill value
  root->val = matrix[root->y][root->x];
  current_explored_val = root->val;
  //cout << root->val << ' ' << current_explored_val << endl;
  //debug Flood Matrix
  /*
  cout << "Flood Matrix" << endl;
  for(int i = 0; i < 5; i++) {
    for(int j = 0; j < 9; j++) {
      cout << matrix[i][j] << ' ';
    }
    cout << endl;
  }
  */
  // ------ modified depth search v2 -------
  //create list to hold visited nodes and queue to hold nodes to be processed
  stack<node*> Q;
  list<node*> visited;
  //start search at start node
  node * current_node = root;
  node * path_list[MAX_PATHS][MAX_STEPS] = {NULL};
  int current_path_i = 0;
  int current_path_j = 0;
  bool path_found = TRUE;
  Q.push(root);
  visited.push_back(root);
  //cout << current_node->x << current_node->y << endl;
  while (Q.empty() == FALSE) {
    while(path_found == TRUE) {
      path_found = FALSE;
      // is there a wall
      if (wall_hori[current_node->y][current_node->x] == FALSE) {
        // is it 1 less than current value?
        if(matrix[current_node->y-1][current_node->x] == current_explored_val-1) {
          // set path found
          path_found = TRUE;
          //create new child node
          current_node->left = new node(current_node->x, current_node->y-1, current_explored_val-1, North);
          //push child node to queue
          Q.push(current_node->left);
        }
      }
      //cout <<'n'<< current_node_i << current_node_j << ' ' << current_x << current_y << endl;
      //check east
      if (wall_vert[current_node->y][current_node->x+1] == FALSE) {
        if(matrix[current_node->y][current_node->x+1] == current_explored_val-1) {
          if (path_found == TRUE) {
            // create second child node
            current_node->right = new node(current_node->x+1, current_node->y, current_explored_val-1, East);
            // push child node to queue
            Q.push(current_node->right);
          }
          else {
            current_node->left = new node(current_node->x+1, current_node->y, current_explored_val-1, East);
            Q.push(current_node->left);
            path_found = TRUE;
          }
        }
      }
      //cout <<'e'<< current_node_i << current_node_j << ' ' << current_x << current_y << endl;
      //check south
      if (wall_hori[current_node->y+1][current_node->x] == FALSE) {
        if(matrix[current_node->y+1][current_node->x] == current_explored_val-1) {
          if (path_found == TRUE) {
            current_node->right = new node(current_node->x, current_node->y+1, current_explored_val-1, South);
            Q.push(current_node->right);
          }
          else {
            current_node->left = new node(current_node->x, current_node->y+1, current_explored_val-1, South);
            path_found = TRUE;
            Q.push(current_node->left);
          }
        }
      }
  //cout << 's' << current_node_i << current_node_j << ' ' << current_x << current_y << endl;
      //check west
      if (wall_vert[current_node->y][current_node->x] == FALSE) {
        if(matrix[current_node->y][current_node->x-1] == current_explored_val-1) {
            // if already path found, duplicate and start new path
          if (path_found == TRUE) {
          // new path
            current_node->right = new node(current_node->x-1, current_node->y, current_explored_val-1, West);
            Q.push(current_node->right);
          }
          else {
            current_node->left = new node(current_node->x-1, current_node->y, current_explored_val-1, West);
            Q.push(current_node->left);
            path_found = TRUE;
          }
        }
      }
      //cout << 'w' << current_node_i << current_node_j << ' ' << current_x << current_y << endl;
      
      if (Q.empty() == FALSE) {
        // set next node to be processed
        current_node = Q.top();
        // push node to visited queue
        visited.push_back(Q.top());
        // remove node from queue
        Q.pop();
      }
      // update explored value to next node
      current_explored_val = matrix[current_node->y][current_node->x];
      // Check to see if the next value is the goal
      if (current_explored_val == 0) {
        //if goal reached, path is complete, copy visited list to path storage array
        for (list<node*>::iterator it = visited.begin(); it!=visited.end(); ++it) {
          path_list[current_path_i][current_path_j] = *it;
          current_path_j++;
          //cout << path_list[current_path_i][current_path_j]->x << path_list[current_path_i][current_path_j]->y << ' ' << path_list[current_path_i][current_path_j]->val << endl;
        }
        //cout << endl;
        // increment path storage ready for next path
        current_path_i++;
        current_path_j = 0;
         //cout << endl << path_list[current_path_i][current_path_j]->x << path_list[current_path_i][current_path_j]->y << endl;   
      }

    }
    //cout << visited.top()->x <<visited.top()->y << endl;
    //cout  << current_node->x << current_node->y << ' ' << current_explored_val << endl;
    // remove from visited, all nodes between current and goal, to define a new path.
    if (Q.empty() == FALSE) {
      bool dup_erase = FALSE;
      for (list<node*>::iterator it = visited.begin(); it!= visited.end(); ) {
          if((*it)->val == current_explored_val  && dup_erase == FALSE) {
            dup_erase = TRUE;
            it = visited.erase(it);
          } else if((*it)->val < current_explored_val) it = visited.erase(it);
        else ++it;
      }
    }
    //cout << current_explored_val << endl;
    path_found = TRUE;
  }
  current_path_i--;
  /*
  for(; current_path_i >= 0; current_path_i--) {
    for(int j = 0; path_list[current_path_i][j] != NULL; j++) {
    cout << path_list[current_path_i][j]->x << path_list[current_path_i][j]->y << ' ' << path_list[current_path_i][j]->val <<endl;
    }
    cout << endl;
  }
  */
  // -------------print paths out---------------//
  wall_vert_i = 0;
  wall_hori_i = 0;
  bool on_path = FALSE;
  int num_path = 1;
  while (current_path_i >= 0) {
    //print horizontal row
    cout << "--- Path " << num_path << " ---" << endl;
    for(int i = 0; i<6; i++) {
      cout << ' ';
      for(int j = 0; j < 9; j++) {
        if (wall_hori[wall_hori_i][j] == TRUE) cout << "--- " ;
        else cout << "    ";
      }
      wall_hori_i++;
      cout << endl;
      //if vertical row to print
      if (wall_vert_i < 5) {
      //if end of verti rows not reached
        cout << '|';
        //first print left wall
        for(int j = 0; j < 9; j++) {
        //iterate through path, check if any path node yx = current cell ij i.e. wall_vert_i j
          for(int k = 0; path_list[current_path_i][k] != NULL; k++) {
            //if it does match, insert its flood value
            if (path_list[current_path_i][k]->x == j && path_list[current_path_i][k]->y == wall_vert_i) {
              if (path_list[current_path_i][k]->val == root->val && path_list[current_path_i][k]->x == root->x &&
                  path_list[current_path_i][k]->y == root->y) {
                  if(root->dir_from_parent == North) cout << ' ' << '^' << ' ';
                  if(root->dir_from_parent == East) cout << ' ' << '>' << ' ';
                  if(root->dir_from_parent == South) cout << ' ' << 'v' << ' ';
                  if(root->dir_from_parent == West) cout << ' ' << '<' << ' ';
              }
              else if(path_list[current_path_i][k]->val < 10) cout << ' ' << path_list[current_path_i][k]->val << ' ';
              else cout << ' ' << path_list[current_path_i][k]->val;
              on_path = TRUE;
              // otherwise blank space
            }
          }
          if(on_path == FALSE)cout << "   ";
          on_path = FALSE;
          // check if wall or not and print accordingly
          if (wall_vert[wall_vert_i][j+1] == TRUE) cout << '|';
          else cout << ' ';
        }
        cout << endl;
      }
      wall_vert_i++;

    }
    num_path++;
    current_path_i--;
    wall_vert_i = 0;
    wall_hori_i = 0;
  }
  num_path--;
  //cout << "Done - " << num_path << " shortest paths found!" << endl;
  
  cout << "--- Task 3 ---" << endl;
  // find shortest number of turns
  int num_dir = 0;
  int shortest_path[MAX_PATHS];
  for(int i = 0; i<10; i++) shortest_path[i] = -1;
  int least_turns = MAX_STEPS;
  Heading prev_heading = path_list[0][0]->dir_from_parent;
  for(int i = 0; path_list[i][0] != NULL; i++) {
    for(int j = 0; path_list[i][j] != NULL; j++) {
      if(path_list[i][j]->dir_from_parent != prev_heading) {
        //cout << prev_heading << ' ' << path_list[i][j]->dir_from_parent << endl;
        num_dir++;
      }
      prev_heading = path_list[i][j]->dir_from_parent;
    }
    if (num_dir <= least_turns) {
      least_turns = num_dir;
      //cout << least_turns << endl;
      
    }
    num_dir = 0;
    prev_heading = path_list[0][0]->dir_from_parent;
  }
  
  //find index of paths with shortest number of turns
  int path_i = 0;
  for(int i = 0; path_list[i][0] != NULL; i++) {
    for(int j = 0; path_list[i][j] != NULL; j++) {
      if(path_list[i][j]->dir_from_parent != prev_heading) {
        //cout << prev_heading << ' ' << path_list[i][j]->dir_from_parent << endl;
        num_dir++;
      }
      prev_heading = path_list[i][j]->dir_from_parent;
    }
    if (num_dir == least_turns) {
      shortest_path[path_i] = i;
      //cout << shortest_path[i] << endl;
      path_i++;
      prev_heading = path_list[0][0]->dir_from_parent;
    }
    num_dir = 0;
    prev_heading = path_list[0][0]->dir_from_parent;
  }
  //least turns debug
  /*
  cout << "shortest paths are at path_list[i]: " << endl;
  for(int i = 0; i < 10; i++) {
    if(shortest_path[i] != -1) {
      cout << shortest_path[i] << endl;
    }
  }
  */
  //----------- print shortest path: -----------

  for(int i = 0; i<6; i++) {
    cout << ' ';
    for(int j = 0; j < 9; j++) {
      if (wall_hori[wall_hori_i][j] == TRUE) cout << "--- " ;
      else cout << "    ";
    }
    wall_hori_i++;
    cout << endl;
    //if vertical row to print
    if (wall_vert_i < 5) {
    //if end of verti rows not reached
      cout << '|';
      //first print left wall
      for(int j = 0; j < 9; j++) {
      //iterate through path, check if any path node yx = current cell ij i.e. wall_vert_i j
        for(int k = 0; path_list[shortest_path[0]][k] != NULL; k++) {
          //if it does match, insert its flood value
          //cout << path_list[current_path_i][k]->val;
          if (path_list[shortest_path[0]][k]->x == j && path_list[shortest_path[0]][k]->y == wall_vert_i) {
            if (path_list[shortest_path[0]][k]->val == root->val && path_list[shortest_path[0]][k]->x == root->x &&
              path_list[shortest_path[0]][k]->y == root->y) {
              if(root->dir_from_parent == North) cout << ' ' << '^' << ' ';
              if(root->dir_from_parent == East) cout << ' ' << '>' << ' ';
              if(root->dir_from_parent == South) cout << ' ' << 'v' << ' ';
              if(root->dir_from_parent == West) cout << ' ' << '<' << ' ';
            }
            else if(path_list[shortest_path[0]][k]->val < 10) cout << ' ' << path_list[shortest_path[0]][k]->val << ' ';
            
            else cout << ' ' << path_list[shortest_path[0]][k]->val;
            on_path = TRUE;
            // otherwise blank space
          }
        }
        if(on_path == FALSE)cout << "   ";
        on_path = FALSE;
        // check if wall or not and print accordingly
        if (wall_vert[wall_vert_i][j+1] == TRUE) cout << '|';
        else cout << ' ';
      }
      cout << endl;
    }
    wall_vert_i++;
  }
  wall_vert_i = 0;
  wall_hori_i = 0;
  

  //----------print first node row/col/dir_from_parent------------
  int num_steps = 0;
  string shortest_path_plan;
  prev_heading = path_list[shortest_path[0]][0]->dir_from_parent;
  shortest_path_plan.push_back(static_cast<char>(path_list[(shortest_path[0])][0]->y+48));
  shortest_path_plan.push_back(static_cast<char>(path_list[(shortest_path[0])][0]->x+48));
  if(path_list[shortest_path[0]][0]->dir_from_parent == North) shortest_path_plan.push_back('N');
  if(path_list[shortest_path[0]][0]->dir_from_parent == East) shortest_path_plan.push_back('E');
  if(path_list[shortest_path[0]][0]->dir_from_parent == South) shortest_path_plan.push_back('S');
  if(path_list[shortest_path[0]][0]->dir_from_parent == West) shortest_path_plan.push_back('W');
  //---------print out path instructions-----------//
  for(int j = 1; path_list[shortest_path[0]][j] != NULL; j++) {
    //cout << path_list[shortest_path[0]][j]->x << path_list[shortest_path[0]][j]->y << ' ' << path_list[shortest_path[0]][j]->dir_from_parent << prev_heading << endl;
    if(abs(path_list[shortest_path[0]][j]->dir_from_parent - prev_heading) == 2) {
      shortest_path_plan.push_back('L');
      shortest_path_plan.push_back('L');
      shortest_path_plan.push_back('F');
      num_steps += 3;
    }
    if(path_list[shortest_path[0]][j]->dir_from_parent - prev_heading == 1 || path_list[shortest_path[0]][j]->dir_from_parent - prev_heading == 3) {
      if(prev_heading == North && path_list[shortest_path[0]][j]->dir_from_parent == West) {
        shortest_path_plan.push_back('L');
        shortest_path_plan.push_back('F');
      }
      else {
        shortest_path_plan.push_back('R');
        shortest_path_plan.push_back('F');
      }
      num_steps += 2;
    }
    if(path_list[shortest_path[0]][j]->dir_from_parent - prev_heading == -1 || path_list[shortest_path[0]][j]->dir_from_parent - prev_heading == -3) {
      if(prev_heading == West && path_list[shortest_path[0]][j]->dir_from_parent == North) {
        shortest_path_plan.push_back('R');
        shortest_path_plan.push_back('F');
      }
      else {
        shortest_path_plan.push_back('L');
        shortest_path_plan.push_back('F');
      }
      num_steps += 2;
    }
    if(path_list[shortest_path[0]][j]->dir_from_parent == prev_heading) {
      shortest_path_plan.push_back('F');
      num_steps++;
    }
    prev_heading = path_list[shortest_path[0]][j]->dir_from_parent;
  }
  cout << "Steps: " << num_steps << endl;
  cout << "Path: " << shortest_path_plan << endl;
  cout << "--- Task 4 ---" << endl;
  // ------------- write out instrutions to file ------------
  cout << "File: " << PATH_PLAN_FILE_NAME << endl;
  ofstream robot_instruct;
  robot_instruct.open(PATH_PLAN_FILE_NAME);
  if (robot_instruct.is_open()) {
    robot_instruct << shortest_path_plan;
    robot_instruct.close();
  }
  else {
    cerr << "Error: Path Plan output file not found" << endl;
    return 1;
  }
  cout << "Path: " << shortest_path_plan << endl;
  // ----- END-----
  return 0;
}