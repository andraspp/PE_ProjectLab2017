#ifndef JHONNY5_MAIN_HH
#define JHONNY5_MAIN_HH

#define _USE_MATH_DEFINES
#include <cmath>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
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

struct orientation {
  float z; // sin (theta / 2)
  float w; // cos (theta / 2)

  orientation(void) : w(0.0f), z(0.0f) {}
  orientation(float _z, float _w) : z(_z), w(_w) {}
};

enum direction {
  DOWN,
  LEFT,
  UP,
  RIGHT
};

#define False (0)
#define True  (1)
//typedef actionlib::SimpleActionServer<jhonny5::MoveOrder> MA;

#define EPSILON 0.005

robot_states_t RobotState;
robot_states_t RobotStateLastLoop;
orientation    robot_orientation;
direction      robot_direction;
bool           FinishTileDetected;
bool           WallDetectedFront;
bool           RobotStopped;
bool           CrossingDetectedLeft;
bool           CrossingDetectedRight;
bool           LocationCheckProcessDone;
float          lsranges[10];
float          avgrange;

void Jhonny5_init(void);
void Jhonny5_input_processing(void);
void Jhonny5_state_machine(void);
void Jhonny5_execute_order(void);
geometry_msgs::Twist turn_left(void);
geometry_msgs::Twist turn_right(void);
orientation get_orienation_from_direction(direction d);
direction get_left_dir(direction d);
direction get_right_dir(direction d);
/* Callback functions */
void finishTileCallback(const std_msgs::BoolConstPtr& str);
void wallProximityCallback(const std_msgs::BoolConstPtr& str);
void crossingLeftCallback(const std_msgs::BoolConstPtr& str);
void crossingRightCallback(const std_msgs::BoolConstPtr& str);
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom);
#endif /* JHONNY5_MAIN_HH */
