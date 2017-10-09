#include "ros/ros.h"
#include "std_msgs/Bool.h" 
#include "std_msgs/String.h" 
#include <actionlib/server/simple_action_server.h>
#include <jhonny5/MoveOrder.h>
#include <sstream>

typedef enum robot_states_e
{
    init = 0;
    position_determination;
    moving;
    end_state;
    error_state;
} robot_states_t;

typedef actionlib::SimpleActionServer<jhonny5::MoveOrder> Server;

robot_states_t RobotState;
robot_states_t RobotStateLastLoop;
Boolean        WallDetectedFront;
Boolean        RobotStopped;