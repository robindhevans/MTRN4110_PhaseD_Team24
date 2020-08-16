// File:          z5204873_MTRN4110_PhaseA.cpp
// Date:          11/06/2020
// Description:   Assignment Phase A
// Author:        Dean So (z5204873)
// Modifications:

// headers
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Compass.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <cmath>
#include <typeinfo>

// definitions
#define PATH_PLAN_FILE_NAME "../../PathPlan.txt"
#define SPEED 6.28*0.3
#define FORWARD_SPEED 6.28
#define PI 3.14158
#define POSITION_STEP_LINEAR 165.0/20.0
#define POSITION_STEP_ROTATION 0.1
#define ROT_TOL 0.01
#define HEADING_TOL 0.01
// All the webots classes are defined in the "webots" namespace
using namespace webots;

double heading_ang_turn(double heading_ang, char turn);
char wall_exists(double wall_reading);
bool heading_almost_equal_edge(double heading_ang, double angle);
bool heading_almost_equal(double heading_ang, double angle);
std::vector<int> update_row_col(std::vector<int> rc_pos, char heading);
char update_heading(char heading, char turn);

int main(int argc, char **argv) {
    // create the Robot instance.
    auto robot = new Robot();

    // get the time step of the current world.
    int timeStep = (int)robot->getBasicTimeStep();
    std::cout << "Time step of " << timeStep << std::endl;
    
    // get the motor devices
    auto leftMotor = robot->getMotor("left wheel motor");
    auto rightMotor = robot->getMotor("right wheel motor");
    
    // get distance sensors
    auto dsF = robot->getDistanceSensor("dsF");
    dsF->enable(timeStep);
    auto dsL = robot->getDistanceSensor("dsL");
    dsL->enable(timeStep);
    auto dsR = robot->getDistanceSensor("dsR");
    dsR->enable(timeStep);
    
    // get wheel sensors
    auto left_encoder = robot->getPositionSensor("left wheel sensor");
    left_encoder->enable(timeStep);
    auto right_encoder = robot->getPositionSensor("right wheel sensor");
    right_encoder->enable(timeStep);
    
    // get inertial unit
    auto imu = robot->getInertialUnit("inertial unit");
    imu->enable(timeStep);
    
    // read pathplan.txt
    std::cout << "Start! Reading Path Plan from \"" << PATH_PLAN_FILE_NAME << "\"" << std::endl;
    std::string pathplan;
    std::ifstream pathplan_txt (PATH_PLAN_FILE_NAME);

    if (pathplan_txt.is_open()) {
        while (getline (pathplan_txt, pathplan)) {
            std::cout << pathplan << std::endl;
        }
        pathplan_txt.close();
    } else {
        std::cout << "Unable to open file" << std::endl;
    }
    std::cout << "Done - Path Plan read!" << std::endl;
    
    std::string::iterator path_it; // iterator to keep track of which turn to execute
    std::string::iterator x;
    // insert wait instructions between each instruction as well as 
    // before first and after last instruction
    for (x = pathplan.begin() + 3; x != pathplan.end(); x += 2) {
        x = pathplan.insert(x, 'W');
    }
    x = pathplan.end();
    x = pathplan.insert(x, 'W');
    //std::cout << pathplan << std::endl;
    
    // point iterator to beginning of pathplan to extract information out of pathplan
    path_it = pathplan.begin();
    // initial row and column position
    int row = *path_it - '0';
    ++path_it;
    int col = *path_it - '0';
    ++path_it;
    std::vector<int> rc_pos {row, col};
    
    // the cardinal direction of the robot
    char heading = *path_it;
    
    // the angular direction of the robot in range -pi to pi
    double heading_ang = 0;
    switch (*path_it) {
        case 'N':
            heading_ang = 0.0;
            break;
        case 'E':
            heading_ang = -PI/2;
            break;
        case 'S':
            heading_ang = PI; // can also be -PI but doesnt matter
            break;
        case 'W':
            heading_ang = PI/2;
            break;
    }
    ++path_it;
    
    
    // setup initial conditions of robot
    
    double left_pos = 0.0;
    double right_pos = 0.0;
    
    double left_encoder_val = 0.0;
    double right_encoder_val = 0.0;
    
    double left_vel = SPEED;
    double right_vel = SPEED;
    
    double dsF_vals;
    double dsL_vals;
    double dsR_vals;
    
    double wait_time = timeStep * 0.001 * 32;
    double forward_time = timeStep * 0.001 * 102 * (SPEED/FORWARD_SPEED);
    double prev_time = 0.0;
    
    // scaling the wait time and forward_time according to current timestep
    wait_time = wait_time*(32/timeStep);
    forward_time = forward_time*(32/timeStep);
    
    // current step of the path plan
    int step = 0;
    
    // state variables used to ensure when entering a state
    // updating the heading is only done once
    bool forward_state = false;
    bool waiting_state = false;
    bool right_state = false;
    bool left_state = false;
    bool display_state = false;
    
    leftMotor->setPosition(left_pos);
    rightMotor->setPosition(right_pos);
    leftMotor->setVelocity(left_vel);
    rightMotor->setVelocity(right_vel);
    
    robot->step(timeStep);
    
    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    std::cout << "Start!" << std::endl;
    while (robot->step(timeStep) != -1) {
        
        //std::cout << "TIME: " << robot->getTime() << "s" << std::endl;
        // interpret pathplan
        //std::cout << "current step: " << *path_it << std::endl;
        
        // Process sensor data here.
        auto imu_data = imu->getRollPitchYaw();
        
        dsF_vals = dsF->getValue();
        dsL_vals = dsL->getValue();
        dsR_vals = dsR->getValue();
        
        left_encoder_val = left_encoder->getValue(); 
        right_encoder_val = right_encoder->getValue();
        
        // set initial position to be equal to initial encoder reading
        left_pos = left_encoder_val;
        right_pos = right_encoder_val;
        
        // print sensor information
        
        std::cout << "roll: " << imu_data[0]
        << " yaw: " << imu_data[1]
        << " pitch: " << imu_data[2] << std::endl;
        
        // std::cout << "dsF: " << dsF_vals << " ";
        // std::cout << "dsL: " << dsL_vals << " ";
        // std::cout << "dsR: " << dsR_vals << " ";
        // std::cout << "left encoder: " << left_encoder_val << " ";
        // std::cout << "right encoder: " << right_encoder_val << std::endl;
        // std::cout << "left pos: " << left_encoder_val << " ";
        // std::cout << "right pos: " << right_encoder_val << std::endl;
        
        
        
        // Process sensor data here
        if (*path_it == 'W') {
            // wait and print current surrounding wall state for robot
            
            if (waiting_state == false) {
                prev_time = robot->getTime();
                //std::cout << "old : " << old << std::endl;
                waiting_state = true;
            }
            if (robot->getTime() < prev_time + wait_time) {
                // keep motors at current position while wall sensing
                if (display_state == false) {
                    std::cout << "Step: ";
                    if (step < 10) std::cout << "0";
                    std::cout << step << ", ";
                    std::cout << "Row: " << rc_pos[0] << ", ";
                    std::cout << "Column: " << rc_pos[1] << ", ";
                    std::cout << "Heading: " << heading << ", ";
                    std::cout << "Left Wall: " << wall_exists(dsL_vals) << ", ";
                    std::cout << "Front Wall: " << wall_exists(dsF_vals) << ", ";
                    std::cout << "Right Wall: " << wall_exists(dsR_vals) << std::endl;
                    display_state = true;
                }
            } else {
                ++path_it;
                waiting_state = false;
                display_state = false;
            }
        } else if (*path_it == 'F') {
            // go forward
            if (forward_state == false) {
                prev_time = robot->getTime();
                //std::cout << "old : " << old << std::endl;
                forward_state = true;
            }
            if (robot->getTime() < prev_time + forward_time) {
                left_pos += POSITION_STEP_LINEAR;
                right_pos += POSITION_STEP_LINEAR;
            } else {
                rc_pos = update_row_col(rc_pos, heading);
                ++path_it;
                ++step;
                forward_state = false;
            }
            // move faster on a forward to save time
            left_vel = FORWARD_SPEED;
            right_vel = FORWARD_SPEED;
        } else if (*path_it == 'L') {
            // turn left
            if (left_state == false) {
                heading_ang = heading_ang_turn(heading_ang, *path_it);
                left_state = true;
            }
            // gyroscopic angle the robot intends to turn to
            std::cout << "LHEADING: " << heading_ang << std::endl;
            // check if heading_ang is PI or -PI
            // on the edge cases of gyroscope
            if (heading_almost_equal_edge(heading_ang, PI)) {
                heading_ang = abs(heading_ang);
                // check if pitch is within negative side and positive side 
                // of boundary within tolerance
                if (imu_data[2] > -heading_ang + ROT_TOL && imu_data[2] > heading_ang - ROT_TOL) {
                    //std::cout << "left" << std::endl;
                    heading = update_heading(heading, *path_it);
                    ++path_it;
                    ++step;
                    left_state = false;
                } else {
                    left_pos -= POSITION_STEP_ROTATION;
                    right_pos += POSITION_STEP_ROTATION;
                }
            } else if (imu_data[2] < heading_ang + ROT_TOL && imu_data[2] > heading_ang - ROT_TOL) {
                //std::cout << "left" << std::endl;
                heading = update_heading(heading, *path_it);
                ++path_it;
                ++step;
                left_state = false;
            } else {
                left_pos -= POSITION_STEP_ROTATION;
                right_pos += POSITION_STEP_ROTATION;
            }
            left_vel = SPEED;
            right_vel = SPEED;
        } else if (*path_it == 'R') {
            // turn right
            if (right_state == false) {
                heading_ang = heading_ang_turn(heading_ang, *path_it);
                right_state = true;
            }
            // gyroscopic angle the robot intends to turn to
            std::cout << "RHEADING: " << heading_ang << std::endl;
            // check if heading_ang is PI or -PI
            // on the edge cases of gyroscope
            if (heading_almost_equal_edge(heading_ang, PI)) {
                heading_ang = abs(heading_ang);
                // check if pitch is within negative side and positive side 
                // of boundary within tolerance
                if (imu_data[2] > -heading_ang + ROT_TOL && imu_data[2] > heading_ang - ROT_TOL) {
                    //std::cout << "right" << std::endl;
                    heading = update_heading(heading, *path_it);
                    ++path_it;
                    ++step;
                    right_state = false;
                } else {
                    left_pos += POSITION_STEP_ROTATION;
                    right_pos -= POSITION_STEP_ROTATION;
                }
            } else if (imu_data[2] < heading_ang + ROT_TOL && imu_data[2] > heading_ang - ROT_TOL) {
                //std::cout << "right" << std::endl;
                heading = update_heading(heading, *path_it);
                ++path_it;
                ++step;
                right_state = false;
            } else {
                left_pos += POSITION_STEP_ROTATION;
                right_pos -= POSITION_STEP_ROTATION;
            }
            left_vel = SPEED;
            right_vel = SPEED;
        } else if (path_it == pathplan.end()){
            // simulation time in seconds
            std::cout << "Finish Time: " << robot->getTime() << std::endl;
            break;
        }
        
        // set the velocity and position of motors
        leftMotor->setVelocity(left_vel);
        rightMotor->setVelocity(right_vel);
        leftMotor->setPosition(left_pos);
        rightMotor->setPosition(right_pos);
    }

    std::cout << "Done - Path plan executed!" << std::endl;
    delete robot;
    return 0;
}

// transforms the heading angle depending on the turn
double heading_ang_turn(double heading_ang, char turn) {
    if (turn == 'L') {
        heading_ang += PI/2;
    } else if (turn == 'R') {
        heading_ang -= PI/2;
    }
    if (heading_ang >= PI) {
        heading_ang -= 2*PI;
    } else if (heading_ang <= -PI) {
        heading_ang += 2*PI;
    }
    return heading_ang;
}

// checks if theres a wall based upon distance sensor readings
char wall_exists(double wall_reading) {
    char wall;
    // if something is detected then its a wall
    if (wall_reading < 1000) {
        wall = 'Y';
    } else {
        wall = 'N';
    }
    return wall;
}

// checks if the heading angle is almost equal to a theoretical angle
// within the heading tolerance on and sign switch boundary condition
// this is specifically used for when pitch of the imu changes from PI to -PI
bool heading_almost_equal_edge(double heading_ang, double angle) {
    bool almost_equal = false;
    if ((heading_ang < angle + HEADING_TOL && heading_ang > angle - HEADING_TOL) || 
        (heading_ang > -angle - HEADING_TOL && heading_ang < -angle + HEADING_TOL)) {
        almost_equal = true;
    }  
    return almost_equal;
}

// checks if the heading angle is almost equal to a theoretical angle
// within the heading tolerance
bool heading_almost_equal(double heading_ang, double angle) {
    bool almost_equal = false;
    if (heading_ang < angle + HEADING_TOL && heading_ang > angle - HEADING_TOL) {
        almost_equal = true;
    }  
    return almost_equal;
}

// updates the row and column position of the robot current cell position 
// using the heading of a current forward movement
std::vector<int> update_row_col(std::vector<int> rc_pos, char heading) {
    if (heading == 'S') {
        rc_pos[0] += 1;
    } else if (heading == 'N') {
        rc_pos[0] -= 1;
    } else if (heading == 'E') {
        rc_pos[1] += 1;
    } else if (heading == 'W') {
        rc_pos[1] -= 1;
    }
    return rc_pos;
}

// updates the cardinal heading direction based upon the turn
char update_heading(char heading, char turn) {
    double cur_angle = 0.0;
    switch (heading) {
        case 'N':
            cur_angle = 0.0;
            break;
        case 'E':
            cur_angle = -PI/2;
            break;
        case 'S':
            cur_angle = PI;
            break;
        case 'W':
            cur_angle = PI/2;
            break;
    }
    
    cur_angle = heading_ang_turn(cur_angle, turn);
    if (heading_almost_equal(cur_angle, 0)) {
        heading = 'N';
    } else if (heading_almost_equal(cur_angle, -PI/2)) {
        heading = 'E';
    } else if (heading_almost_equal_edge(cur_angle, PI)) {
        heading = 'S';
    } else if (heading_almost_equal(cur_angle, PI/2)) {
        heading = 'W';
    } 
    return heading;
}