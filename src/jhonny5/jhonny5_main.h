#ifndef JHONNY5_MAIN_HH
#define JHONNY5_MAIN_HH

#include "ros/ros.h"
#include "std_msgs/Bool.h" 
#include "std_msgs/String.h" 
#include <geometry_msgs/Twist.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/LaserScan.h>
#include <gazebo/gazebo.hh>
#include <sstream>

typedef enum robot_states_e
{
    init = 0,
    position_determination,
    moving,
    end_state,
    error_state
} robot_states_t;

#define False (0)
#define True  (1)
//typedef actionlib::SimpleActionServer<jhonny5::MoveOrder> MA;

robot_states_t RobotState;
robot_states_t RobotStateLastLoop;
bool           FinishTileDetected;
bool           WallDetectedFront;
bool           RobotStopped;
bool           CrossingDetectedLeft;
bool           CrossingDetectedRight;
bool           LocationCheckProcessDone;
float          lsranges[10];
float          avgrange;
unsigned int   StopTimer;
unsigned int   TurnTimer;
bool           TurnFinished;

void Jhonny5_init(void);
void Jhonny5_input_processing(void);
void Jhonny5_state_machine(void);
void Jhonny5_execute_order(void);
/* Callback functions */
void finishTileCallback(const std_msgs::BoolConstPtr& str);
void wallProximityCallback(const std_msgs::BoolConstPtr& str);
void crossingLeftCallback(const std_msgs::BoolConstPtr& str);
void crossingRightCallback(const std_msgs::BoolConstPtr& str);
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
#endif /* JHONNY5_MAIN_HH */