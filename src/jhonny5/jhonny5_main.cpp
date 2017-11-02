#include "jhonny5_main.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "jhonny5_state_machine");

    Jhonny5_init();
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub_;
    ros::Rate loop_rate(10);
    geometry_msgs::Twist base_cmd;
    
    ros::Subscriber finishTileCheck = nh.subscribe("/jhonny5/finishtileinfo", 1, finishTileCallback);
    ros::Subscriber wallProximityCheck = nh.subscribe("/jhonny5/wallproximityinfo", 1, wallProximityCallback);
    ros::Subscriber crossingLeftCheck = nh.subscribe("/jhonny5/crossingleftinfo", 1, crossingLeftCallback);
    ros::Subscriber crossingRightCheck = nh.subscribe("/jhonny5/crossingrightinfo", 1, crossingRightCallback);
    ros::Subscriber laserScanCheck = nh.subscribe<sensor_msgs::LaserScan>("/jhonny5/laser/scan",10,&laserScanCallback);
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;

    while(ros::ok())
    {
        if(avgrange > 2.5)
        {
            base_cmd.angular.z = 1.00;
            base_cmd.linear.x = base_cmd.linear.y = 0;
        }
        else
        {
            base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
        }

        Jhonny5_state_machine();
        Jhonny5_execute_order();        

        cmd_vel_pub_.publish(base_cmd);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void Jhonny5_init(void)
{
    RobotState = init;
    RobotStateLastLoop = RobotState;
    FinishTileDetected = False;
    WallDetectedFront = False;
    CrossingDetectedLeft = False;
    CrossingDetectedRight = False;
    LocationCheckProcessDone = False;
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
        if(FinishTileDetected == True)
        {
            RobotState = end_state;
        }
        else if(LocationCheckProcessDone == True)
        {
            LocationCheckProcessDone = False;
            RobotState = moving;
        }
        break;
    case moving:
        if(   (WallDetectedFront == True)
           || (CrossingDetectedLeft == True)
           || (CrossingDetectedRight == True)
           || (FinishTileDetected == True)
          )
        {
            RobotState = position_determination;
        }
        break;
    case end_state:
        break;
    case error_state:
    default:
        RobotState = error_state;
        break;
    }

    /* State dependent actions */
    if(RobotStateLastLoop == RobotStateLastLoop)
    {
        switch(RobotState)
        {
        case init:
            break;
        case position_determination:
            break;
        case moving:
            break;
        case end_state:
            break;
        case error_state:
        default:
            ros::shutdown();
            break;
        }
    }
    else
    {
        switch(RobotStateLastLoop)
        {
        case moving:
            if(RobotState == position_determination)
            break;
        default:
            break;   
        }
    }
}

void Jhonny5_execute_order(void)
{

}

void finishTileCallback(const std_msgs::BoolConstPtr& str)
{
    FinishTileDetected = (bool)str;
}

void wallProximityCallback(const std_msgs::BoolConstPtr& str)
{
    WallDetectedFront = (bool)str;
}

void crossingLeftCallback(const std_msgs::BoolConstPtr& str)
{
    CrossingDetectedLeft = (bool)str;
}

void crossingRightCallback(const std_msgs::BoolConstPtr& str)
{
    CrossingDetectedRight = (bool)str;
}

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
     int i;
     float sum = 0;
     avgrange = 0;
     for(i=0; i<10; i++)
     {
         lsranges[i] = scan->ranges[i];
         sum += lsranges[i];
         printf("Range data %d : %1.8f\n",i, lsranges[i]);
     }
     avgrange = sum/10;
     printf("Average: %1.8f\n",avgrange);
}