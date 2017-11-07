#ifndef JHONNY5_MAIN_HH
#define JHONNY5_MAIN_HH

#include <ros/ros.h>
#include <std_msgs/Bool.h> 
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/LaserScan.h>
#include <gazebo/gazebo.hh>
#include <tf/tf.h>
#include <sstream>

typedef enum robot_states_e
{
    init = 0,
    position_determination,
    moving
} robot_states_t;

typedef enum path_selection_e
{
    left = 0,
    front = 1,
    right = 2,
    turnaround = 3,
    none = 5
} path_selection_t;

#define wallCloseThreshold (2.00)
#define wallCloseOffset (5.00)
#define forwardSpeed (1.00)
#define turnSpeed (0.25)
#define turnSpeedSlow (0.1)
#define stopSpeed (0.00)
#define crossConfirmedTime (20)

robot_states_t RobotState;
robot_states_t RobotStateLastLoop;
bool           WallDetectedFront;
bool           RobotStopped;
bool           CrossingDetectedLeft;
bool           CrossingDetectedLeftLL;
bool           CrossingDetectedRight;
bool           CrossingDetectedRightLL;
float          LsFrontRanges[10];
float          LsLeftRanges[10];
float          LsRightRanges[10];
float          AvgFrontRange;
float          AvgLeftRange;
float          AvgRightRange;
bool           TurnFinished;
path_selection_t ChosenPath;
bool           PathSelected;
bool           SuppressPathUpdate;
unsigned int   CrossConfirmTimerLeft;
unsigned int   CrossConfirmTimerRight;
double         YawCurrent;
double         RollCurrent;
double         PitchCurrent;

ros::Publisher cmd_vel_pub_;
ros::Publisher wall_prox_pub_;
ros::Publisher cross_left_pub_;
ros::Publisher cross_right_pub_;

/* Main functions*/
void Jhonny5_init(void);
void Jhonny5_state_machine(void);
void set_velocities(float lin_vel, float ang_vel);
void turn(float angleDeg, double angularSpeed, int direction);
/* Callback functions */
void laserScanFrontCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
void laserScanLeftCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
void laserScanRightCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
void getOdom(const nav_msgs::Odometry::ConstPtr& msg);
#endif /* JHONNY5_MAIN_HH */