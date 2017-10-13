#include "jhonny5_main.h"

int main(int argc, char **argv)
{
    ros::Rate loop_rate(10);
    ros::init(argc, argv, "state_machine");

    unsigned int num_readings = 100;
    double ranges[num_readings];
    double laser_frequency = 40;
    double intensities[num_readings];

    //RobotState = init;
    RobotStopped = False;
    wallClose = False;

    ros::NodeHandle n;
    //ros::Subscriber sub = n.subscribe("laserScannerFront", 10, scannerFrontCallback);
    ros::Publisher cmd_vel_pub_;

    cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    //Server stopRobotServer(n, "stop_robot", boost::bind(&executeStopRobot, _1, &server), false);
    
    geometry_msgs::Twist base_cmd;
    base_cmd.linear.x = 0.25;
    base_cmd.linear.y = base_cmd.angular.z = 0;


    while(ros::ok())
    {
        ros::Time scan_time = ros::Time::now();
       

        if(wallClose == True)
        {
            base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
            RobotStopped = True;
        }

        if(RobotStopped == True)
        {
            ros::shutdown();
        }
        
        cmd_vel_pub_.publish(base_cmd);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

//void scannerFrontCallback(const std_msgs::Bool::ConstPtr& wallClose)
//{
//    ((wallClose == True) ? (WallDetectedFront = True) : (WallDetectedFront = False));
//}