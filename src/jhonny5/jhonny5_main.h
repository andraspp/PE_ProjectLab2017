#include "ros/ros.h"
#include "std_msgs/Bool.h" 
#include "std_msgs/String.h" 
#include <geometry_msgs/Twist.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/LaserScan.h>
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
//typedef actionlib::SimpleActionServer<jhonny5::MoveOrder> Server;

robot_states_t RobotState;
robot_states_t RobotStateLastLoop;
int            WallDetectedFront;
int            RobotStopped;
