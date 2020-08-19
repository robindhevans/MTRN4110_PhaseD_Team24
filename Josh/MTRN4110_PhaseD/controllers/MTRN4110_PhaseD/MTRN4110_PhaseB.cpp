// File: z5195019_MTRN4110_PhaseB.cpp
// Date: Started 25/6/2020
// Description: Path Planning
// Author: Josh Purnell
// Modifications:


#include <webots/Robot.hpp>
#include <fstream>
#include <string>
#include <iterator>
#include <vector>

#define TIME_STEP 64
#define MAP_TEXT_COLS 37
#define MAP_TEXT_ROWS 11
#define MAP_GOAL_ROW 2
#define MAP_GOAL_COL 4
#define MAP_COLS 9
#define MAP_ROWS 5
#define MAP_MAX 45
#define MAP_FILE_NAME "../../Map.txt"
#define PATH_OUTPUT "../../PathPlanFound.txt"


using namespace webots;
using namespace std;

typedef struct robotdata robotdata;
struct robotdata {
  int rowindex;
  int colindex;
  int pathvalue;
  char heading;
  };
  
typedef struct mapdata mapdata;
struct mapdata {
  char text[MAP_TEXT_ROWS][MAP_TEXT_COLS];
  int cellvalue[MAP_ROWS][MAP_COLS];
  int hwalls[MAP_ROWS + 1][MAP_COLS];
  int vwalls[MAP_ROWS][MAP_COLS + 1];
  
};

typedef struct stepdata stepdata;
struct stepdata{
  vector <char> instr;
  int heading;
  //North=0 - East=1 - South=2 - West=3
};

typedef struct finalpath finalpath;
struct finalpath{
  vector <char> instr;
  char map[MAP_TEXT_ROWS][MAP_TEXT_COLS];
};
  
int next_step(int pathtree[MAP_ROWS][MAP_COLS] , mapdata map, stepdata step, finalpath* route, int cur_dist, int rowpos, int colpos, int& path_count);
int getheading(char robheading);
char convertheading(int int_heading);
int writeinstr(stepdata& step, int direction);

int MTRN4110_PhaseB(char maptext[][MAP_TEXT_COLS], char pathPlan[]) {
  mapdata map;
  stepdata step;
  finalpath shortestpath;
  finalpath* route = &shortestpath; 
  int i;
  int j;
  int rowwall = 0;
  int colwall = 0;
  
  
  for(int i = 0; i < MAP_TEXT_ROWS; i++){
    for(int j = 0; i < MAP_TEXT_COLS;i++){
      map.text[i][j] = maptext[i][j];
    }
  }

  
  cout << "--- Task 2 ---" << endl;
  
  //determine to horizontal wall array
  i = 0;
  j = 2;

  //cout << "Horizontal wall array" << endl;
  for(i = 0; i< MAP_TEXT_ROWS; i += 2){
    colwall = 0;
    for (j = 2; j < MAP_TEXT_COLS; j +=4){
      if (map.text[i][j] == '-'){
        map.hwalls[rowwall][colwall] = 1;
      }
      else{
        map.hwalls[rowwall][colwall] = 0;
      }
      //cout <<  map.hwalls[rowwall][colwall] << " "; // hoizontal wall array printout
      colwall++;
    }
    //cout << endl;
    rowwall++;
   }
   
  //determine to vertical wall array
  i = 1;
  j = 0;
  rowwall = 0;
  //cout << "Vertical wall array" << endl;
  for (i = 1; i < MAP_TEXT_ROWS; i += 2){
    colwall = 0;
    for (j = 0; j < MAP_TEXT_COLS; j += 4){
      if (map.text[i][j] == '|'){
        map.vwalls[rowwall][colwall] = 1;
      }
      else{
        map.vwalls[rowwall][colwall] = 0;
      }
      //cout << map.vwalls[rowwall][colwall] << " "; // vertical wall array printout
      colwall++;
    }
    //cout << endl;
    rowwall++;
  } 
  
  
  //Flood Fill Algorithm
  
  //Initialize array to zero
  for (i = 0 ; i < MAP_ROWS ; i++){
    for (j = 0 ; j < MAP_COLS ; j++){
      map.cellvalue[i][j] = 45;
    }
  } 
 
  
  map.cellvalue[MAP_GOAL_ROW][MAP_GOAL_COL] = 0;
  int dist = 0;
  int mapchange = 1;
 
  // flood array with cell distance values
  while (mapchange){
    mapchange = 0;
    for (i = 0 ; i < MAP_ROWS ; i++){
      for (j = 0 ; j < MAP_COLS ; j++){
        if (map.cellvalue[i][j] == dist) {
        //north
          if (i > 0 && !map.hwalls[i][j] && map.cellvalue[i-1][j] == MAP_MAX){
            map.cellvalue[i-1][j] = map.cellvalue[i][j] + 1;
            mapchange = 1;
          }
         //south
          if (i < 4 && !map.hwalls[i+1][j] && map.cellvalue[i+1][j] == MAP_MAX){
            map.cellvalue[i+1][j] = map.cellvalue[i][j] + 1;
            mapchange = 1;
          }  
          //east
          if ( j < 8 && !map.vwalls[i][j+1] && map.cellvalue[i][j+1] == MAP_MAX){
            map.cellvalue[i][j+1] = map.cellvalue[i][j] + 1;
            mapchange = 1;
          }   
          //west
          if (j > 0 && !map.vwalls[i][j] && map.cellvalue[i][j-1] == MAP_MAX){
            map.cellvalue[i][j-1] = map.cellvalue[i][j] + 1;
            mapchange = 1;
          }
        }
      }
    }
    dist++;
  }
  
  //Robot position
  robotdata robopos;
  for (i = 1; i < MAP_TEXT_ROWS; i += 2){
    for (j = 2; j < MAP_TEXT_COLS; j += 4){
      if (map.text[i][j] != ' '){
        robopos.rowindex = (i-1)/2;
        robopos.colindex = (j-2)/4;
        robopos.pathvalue = map.cellvalue[robopos.rowindex][robopos.colindex];
        step.heading = getheading (map.text[i][j]);
      }
    }
  } 
  
 
  // PATH PLANNING
  int patharray[MAP_ROWS][MAP_COLS];
  
   //Initialize array
  for (i = 0 ; i < MAP_ROWS ; i++){
    for (j = 0 ; j < MAP_COLS ; j++){
      patharray[i][j] = -1;
    }
  } 

  int cur_dist = robopos.pathvalue;
  int rowpos = robopos.rowindex;
  int colpos = robopos.colindex;
  patharray[rowpos][colpos] = map.cellvalue[rowpos][colpos];
  
  int path_total = 1;
  //recursive function finds all path options
  cur_dist = next_step(patharray, map, step, route, cur_dist, rowpos, colpos, path_total);
  
  cout << "--- Task 3 ---" << endl;
  for(int i = 0; i < MAP_TEXT_ROWS; i++){
    for(int j = 0; j < MAP_TEXT_COLS; j++){
      cout << route->map[i][j];
    }
    cout << endl;
  }
  
  //add starting position details
  //convert int to N,S,E,W
  robopos.heading = convertheading(step.heading);
  route->instr.emplace(route->instr.begin(), robopos.heading);
  //insert starting coordinates by converting ints to chars 
  //(using ascii numeric character offset)
  route->instr.emplace(route->instr.begin(), colpos+48);
  route->instr.emplace(route->instr.begin(), rowpos+48);

  cout << "Steps: " << route->instr.size()-3 << endl;
  cout << "Path: ";
  vector<char>::iterator it = route->instr.begin();
  int index = 0;
  for(it = route->instr.begin(); it != route->instr.end(); ++it){
    cout << *it;
    pathPlan[index] = *it;
    pathPlan++; 
  }
  cout << endl;
  cout << "--- Task 4 ---" << endl;
  
  //open and clear file
  ofstream pathfile(PATH_OUTPUT, ofstream::out);
  //print to text file;
  
   if (!pathfile.is_open()) {
    cout << "Failed to pathplan" << endl;
    cout << "Exiting Controller..." << endl;
    exit(EXIT_FAILURE);
  } 
    
  cout << "File: " << PATH_OUTPUT << endl;
  
  cout << "Path: ";
  
  //vector<char>::iterator it = step.instr.begin();
  for(it = route->instr.begin(); it != route->instr.end(); ++it){
    pathfile << *it;
    cout << *it;
  }
  pathfile << endl;
  cout << endl;
   
  pathfile.close();
  return 0;
}

//FUNCTIONS
//converts heading from symbol to integer
int getheading(char robheading){
  int result;
  if(robheading == '^') result = 0;
  else if(robheading == '>') result = 1;
  else if(robheading == 'v') result = 2;
  else if(robheading == '<') result = 3;
  return result;
}
// converts heading from integer to char
char convertheading(int int_heading){
  char result;
  if(int_heading == 0) result = 'N';
  else if(int_heading  == 1) result = 'E';
  else if(int_heading == 2) result = 'S';
  else if(int_heading == 3) result = 'W';
  return result;
}

//writes the instruction data to an array
int writeinstr(stepdata& step, int direction){
  //int newheading;
  if(step.heading == direction){
    step.instr.push_back('F');
    //newheading = direction;
  }else if((step.heading == 0 && direction == 3) || step.heading-1 == direction){
    step.instr.push_back('L');
    step.instr.push_back('F');
    //newheading = direction;
  }else if((step.heading == 3 && direction == 0) || step.heading+1 == direction){
    step.instr.push_back('R');
    step.instr.push_back('F');
    //newheading = direction;
  }
  return direction;
}
    
//recursive function for path finding  
int next_step(int patharray[MAP_ROWS][MAP_COLS], mapdata map, stepdata step, finalpath* route, int cur_dist, int rowpos, int colpos, int& path_count){
  
  //if at the centre print out route.
  if (cur_dist == 0){
    int shortest = 0;
    //check if this is the shortest route so far.
    //if so save in refgerenced variable
    if(route->instr.size() == 0 || step.instr.size() < route->instr.size()){
      shortest = 1;
      route->instr.clear();
      vector<char>::iterator it;
      for(it = step.instr.begin(); it != step.instr.end(); ++it){
        route->instr.push_back(*it);
      }
    }  
   
    
    rowpos = MAP_GOAL_ROW;
    colpos = MAP_GOAL_COL;
    cout << "--- Path - " << path_count << " ---" << endl;
    for(int i = 0; i < MAP_ROWS; i++){
      for(int j = 0; j < MAP_COLS; j++){
        if (map.text[(i*2)+1][(j*4)+2] == ' '){
          if (patharray[i][j] >= 0) {
            if(patharray[i][j] > 9){
              string num = to_string(patharray[i][j]);
              map.text[(i*2)+1][(j*4)+2] = num[0];
              map.text[(i*2)+1][(j*4)+3] = num[1];
            }else {
              string num = to_string(patharray[i][j]);
              map.text[(i*2)+1][(j*4)+2] = num[0];
            }
          }
        }
      }
    }
    for(int i = 0; i < MAP_TEXT_ROWS; i++){
      for(int j = 0; j < MAP_TEXT_COLS; j++){
        cout << map.text[i][j];
        if(shortest){
          route->map[i][j] = map.text[i][j];
        }
      }
    cout << endl;
    }
    

    path_count++;
    return ++cur_dist;
  }
    
   //otherwise check all different directions
   //north
    if (rowpos > 0 && !map.hwalls[rowpos][colpos] && map.cellvalue[rowpos-1][colpos] == cur_dist-1){
      rowpos--;
      int temp = step.heading;
      patharray[rowpos][colpos] = map.cellvalue[rowpos][colpos];
      step.heading = writeinstr(step, 0);      
      cur_dist = next_step(patharray, map, step, route, cur_dist-1, rowpos, colpos, path_count);
      if (temp != step.heading){
        step.instr.pop_back();
      }
      step.instr.pop_back();
      step.heading = temp;
      rowpos++;
    }
    //south
    if (rowpos < 4 && !map.hwalls[rowpos+1][colpos] && map.cellvalue[rowpos+1][colpos] == cur_dist-1){
      rowpos++;
      int temp = step.heading;
      patharray[rowpos][colpos] = map.cellvalue[rowpos][colpos];
      step.heading = writeinstr(step, 2);
      cur_dist = next_step(patharray, map, step, route, cur_dist-1, rowpos, colpos, path_count);
      if (temp != step.heading){
        step.instr.pop_back();
      }
      step.instr.pop_back();
      step.heading = temp;
      rowpos--;
    }
    //east
    if (colpos < 8 && !map.vwalls[rowpos][colpos+1] && map.cellvalue[rowpos][colpos+1] == cur_dist-1){
      colpos++;
      int temp = step.heading;
      patharray[rowpos][colpos] = map.cellvalue[rowpos][colpos];
      step.heading = writeinstr(step, 1);
      cur_dist = next_step(patharray, map, step, route, cur_dist-1, rowpos, colpos, path_count);
      if (temp != step.heading){
        step.instr.pop_back();
      }
      step.instr.pop_back();
      step.heading = temp;
      colpos--;
    }
    //west
    if (colpos > 0 && !map.vwalls[rowpos][colpos] && map.cellvalue[rowpos][colpos-1] == cur_dist-1){
      colpos--;
      int temp = step.heading;
      patharray[rowpos][colpos] = map.cellvalue[rowpos][colpos];
      step.heading = writeinstr(step, 3);
      cur_dist = next_step(patharray, map, step, route, cur_dist-1, rowpos, colpos, path_count);
      if (temp != step.heading){
        step.instr.pop_back();
      }
      step.instr.pop_back();
      step.heading = temp;
      colpos++;
    }
    //clean array on the way back up.
    if (cur_dist > 0){
      patharray[rowpos][colpos] = -1;
    }
    return ++cur_dist;
  }


  