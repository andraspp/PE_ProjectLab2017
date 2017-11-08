#include "jhonny5_main.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "jhonny5_state_machine");

    Jhonny5_init();
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);
    geometry_msgs::Twist base_cmd;

    ros::Subscriber odometryCheck = nh.subscribe<nav_msgs::Odometry>("/jhonny5/odom", sizeof(nav_msgs::Odometry), &odometryCallback);
    ros::topic::waitForMessage<nav_msgs::Odometry>("/jhonny5/odom");
    ros::Subscriber laserScanFrontCheck = nh.subscribe<sensor_msgs::LaserScan>("/jhonny5/laser/scan",10,&laserScanFrontCallback);
    ros::Subscriber laserScanLeftCheck = nh.subscribe<sensor_msgs::LaserScan>("/jhonny5/laser/scan_left",10,&laserScanLeftCallback);
    ros::Subscriber laserScanRightCheck = nh.subscribe<sensor_msgs::LaserScan>("/jhonny5/laser/scan_right",10,&laserScanRightCallback);

    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    set_velocities(stopSpeed, stopSpeed);

    bool turnOnce = true;

    while(ros::ok())
    {
        Jhonny5_state_machine();

        ROS_INFO("Front dist average: %1.8f",AvgFrontRange);
        ROS_INFO("Left dist average: %1.8f",AvgLeftRange);
        ROS_INFO("Right dist average: %1.8f",AvgRightRange);
        ROS_INFO("Position determination: %d, %d, %d", CrossingDetectedLeft, WallDetectedFront, CrossingDetectedRight);
        ROS_INFO("RobotState: %d", RobotState);
        ROS_INFO("ChosenPath: %d", ChosenPath);
        ROS_INFO("Path has been chosen: %d", PathSelected);
        ROS_INFO("Path update suppressed: %d", SuppressPathUpdate);
        ROS_INFO("Turn finished: %d\n\n", TurnFinished);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void Jhonny5_init(void)
{
    RobotState = init;
    RobotStateLastLoop = RobotState;

    robot_direction = DOWN;
    WallDetectedFront = false;
    CrossingDetectedLeft = false;
    CrossingDetectedLeftLL = CrossingDetectedLeft;
    CrossingDetectedRight = false;
    CrossingDetectedRightLL = CrossingDetectedRight;
    TurnFinished = false;
    SuppressPathUpdate = false;
    PathSelected = false;
    ChosenPath = none;
    SuppressPathUpdateTimer = 0;
}

void Jhonny5_state_machine(void)
{
    RobotStateLastLoop = RobotState;

    /* State switch conditions */
    switch(RobotState)
    {
    case init:
        RobotState = position_determination;
        break;
    case position_determination:
        if(   (PathSelected == true)
           && (ChosenPath != front)
           && (ChosenPath != none)
          )
        {
            RobotState = turning;
        }
        else if(   (PathSelected == true)
                && (ChosenPath == front)
                && (WallDetectedFront == false)
               )
        {
            RobotState = moving;
        }
        else
        {
            RobotState = position_determination;
        }
        break;
    case moving:
        if(   (SuppressPathUpdate == false)
           && (   (WallDetectedFront == true)
               || (CrossingDetectedRight == true)
              )
          )
        {
            RobotState = position_determination;
        }
        else
        {
            RobotState = moving;
        }
        break;
    case turning:
        if(TurnFinished == true)
        {
            RobotState = moving;
        }
        else
        {
            RobotState = turning;
        }
        break;
    default:
        RobotState = position_determination;
        break;
    }

    /* State dependent actions */
    if(RobotStateLastLoop != RobotState)
    {
        switch(RobotStateLastLoop)
        {
        case position_determination:
            TurnFinished = false;
            break;
        case moving:
            PathSelected = false;
            break;
        case turning: 
            break;
        default:
            break;
        }
    }
    else
    {
        switch(RobotState)
        {
        case position_determination:
            if(CrossingDetectedRight == true)
            {
                ChosenPath = right;
                SuppressPathUpdateTimer = suppressionTime;
            }
            else if(WallDetectedFront == false)
            {
                ChosenPath = front;
                SuppressPathUpdateTimer = 0;
            }
            else if(CrossingDetectedLeft == true)
            {
                ChosenPath = left;
                SuppressPathUpdateTimer = suppressionTime;
            }
            else
            {
                ChosenPath = turnaround;
                SuppressPathUpdateTimer = 0;
            }
            PathSelected = true;
            SuppressPathUpdate = true;
            
            set_velocities(stopSpeed, stopSpeed);
            break;
        case moving:
            set_velocities(stopSpeed, forwardSpeed);
            PathSelected = none;

            if(SuppressPathUpdateTimer > 0)
            {
                SuppressPathUpdateTimer--;
            }
            else
            {
                SuppressPathUpdate = false;
            }
            break;
        case turning:
            switch(ChosenPath)
            {
            case left:
                turn_left();
                break;
            case front:
                TurnFinished = true;
                break;
            case right:
                turn_right();
                break;
            case turnaround:
                turn_left();
                turn_left();
                break;
            default:
                break;
            }
            TurnFinished = true;
            break;
        default:
            break;
        }
    }
    
}

void set_velocities(float lin_vel, float ang_vel)
{
    geometry_msgs::Twist msg;
    msg.linear.x = lin_vel;
    msg.angular.z = ang_vel;

    cmd_vel_pub_.publish(msg);
}

void laserScanFrontCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
     int i;
     float sumfront = 0;

     for(i=0; i<10; i++)
     {
         LsFrontRanges[i] = scan->ranges[i];
         sumfront += LsFrontRanges[i];
         //printf("Front range data %d : %1.8f\n",i, LsFrontRanges[i]);
     }
     //if (RobotState == moving) //freeze while turning
     {
        AvgFrontRange = sumfront/10;
     }

     if(   (AvgFrontRange > (wallCloseThreshold + wallCloseOffset))
        || (RobotState == turning)
       )
     {
        WallDetectedFront = false;
     }
     else if(AvgFrontRange < wallCloseThreshold)
     {
        WallDetectedFront = true;
     }
     else{}
     //ROS_INFO("Front average: %1.8f\n",AvgFrontRange);
     //ROS_INFO("Wall Front: %d\n", WallDetectedFront);
}

void laserScanLeftCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
     int j;
     float sumleft = 0;

     for(j=0; j<10; j++)
     {
         LsLeftRanges[j] = scan->ranges[j];
         sumleft += LsLeftRanges[j];
         //printf("Left range data %d : %1.8f\n",j, LsLeftRanges[j]);
     }
     //if (RobotState == moving) //freeze while turning
     {
        AvgLeftRange = sumleft/10;
     }

     CrossingDetectedLeftLL = CrossingDetectedLeft;

     if(   (AvgLeftRange <= wallCloseThreshold)
        || (RobotState == turning)
       )
     {
         CrossingDetectedLeft = false;
         CrossConfirmTimerLeft = crossConfirmedTime;
     }
     else if(AvgLeftRange >= (wallCloseThreshold + wallCloseOffset))
     {
         if(   (CrossConfirmTimerLeft > 0)
            && (WallDetectedFront == false)
           )
         {
             CrossConfirmTimerLeft--;
         }
         else
         {
             CrossingDetectedLeft = true;
         }
     }
     else{}

     //ROS_INFO("Left average: %1.8f\n",AvgLeftRange);
     //ROS_INFO("CrossLeft: %d\n", CrossingDetectedLeft);
}

void laserScanRightCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
     int k;
     float sumright = 0;

     for(k=0; k<10; k++)
     {
         LsRightRanges[k] = scan->ranges[k];
         sumright += LsRightRanges[k];
         //printf("Right range data %d : %1.8f\n",k, LsRightRanges[k]);
     }
     //if (RobotState == moving) //freeze while turning
     {
        AvgRightRange = sumright/10;
     }

     CrossingDetectedRightLL = CrossingDetectedRight;

     
     if(   (AvgRightRange <= wallCloseThreshold)
        || (RobotState == turning)
       )
     {
         CrossingDetectedRight = false;
         CrossConfirmTimerRight = crossConfirmedTime;
     }
     else if(AvgRightRange >= (wallCloseThreshold + wallCloseOffset))
     {
         if(   (CrossConfirmTimerRight > 0)
            && (WallDetectedFront == false)
           )
         {
             CrossConfirmTimerRight--;
         }
         else
         {
             CrossingDetectedRight = true;
         }
     }
     else{}
     //ROS_INFO("Right average: %1.8f\n",AvgRightRange);
     //ROS_INFO("CrossRight: %d\n", CrossingDetectedRight);
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom) {
  robot_orientation.z = odom->pose.pose.orientation.z;
  robot_orientation.w = odom->pose.pose.orientation.w;
}

geometry_msgs::Twist turn_left(void) {
  direction   dir_target = get_left_dir(robot_direction);
  orientation or_target  = get_orienation_from_direction(dir_target);
  geometry_msgs::Twist twist;
  ros::Rate rate(10);
  char* str_directions[] = {
    "DOWN",
    "LEFT",
    "UP",
    "RIGHT"
  };
  ROS_INFO("Target direction: %s \n", str_directions[dir_target]);


  while (!((fabs(or_target.z - robot_orientation.z) < EPSILON && fabs(or_target.w - robot_orientation.w) < EPSILON) ||
      (fabs(or_target.z + robot_orientation.z) < EPSILON && fabs(or_target.w + robot_orientation.w) < EPSILON))) {
    // printf("almafa: (%f, %f), (%f, %f) \n", or_target.z,  or_target.w,  robot_orientation.z, robot_orientation.w);
    twist.linear.x = 0.1;
    cmd_vel_pub_.publish(twist);

    ros::spinOnce();
    rate.sleep();
  }

  twist.linear.x = 0.0;
  cmd_vel_pub_.publish(twist);

  robot_direction = dir_target;
  ROS_INFO("Finshed turning new direction: %s \n", str_directions[robot_direction]);
  return twist;
}

geometry_msgs::Twist turn_right(void) {
  direction   dir_target = get_right_dir(robot_direction);
  orientation or_target  = get_orienation_from_direction(dir_target);
  geometry_msgs::Twist twist;
  ros::Rate rate(10);
  char* str_directions[] = {
    "DOWN",
    "LEFT",
    "UP",
    "RIGHT"
  };
ROS_INFO("Target direction: %s \n", str_directions[dir_target]);

  while (!((fabs(or_target.z - robot_orientation.z) < EPSILON && fabs(or_target.w - robot_orientation.w) < EPSILON) ||
      (fabs(or_target.z + robot_orientation.z) < EPSILON && fabs(or_target.w + robot_orientation.w) < EPSILON))) {
    twist.linear.x = -0.1;
    cmd_vel_pub_.publish(twist);

    ros::spinOnce();
    rate.sleep();
  }

  twist.linear.x = 0.0;
  cmd_vel_pub_.publish(twist);

  robot_direction = dir_target;
  ROS_INFO("Finshed turning new direction: %s \n", str_directions[robot_direction]);

  return twist;
}

orientation get_orienation_from_direction(direction d) {
  switch (d) {
    case DOWN:
      return orientation(sin(2.0 * M_PI / 2.0), cos(2.0 * M_PI / 2.0));

    case LEFT:
      return orientation(sin(1.5 * M_PI / 2.0), cos(1.5 * M_PI / 2.0));

    case UP:
      return orientation(sin(1.0 * M_PI / 2.0), cos(1.0 * M_PI / 2.0));

    case RIGHT:
      return orientation(sin(0.5 * M_PI / 2.0), cos(0.5 * M_PI / 2.0));
  }
}

direction get_left_dir(direction d) {
  return (direction)((d + 3) % 4);
}

direction get_right_dir(direction d) {
  return (direction)((d + 1) % 4);
}
