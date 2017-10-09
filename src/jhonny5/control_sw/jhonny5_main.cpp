#include "jhonny5_main.h"

int main(int argc, char **argv)
{
    ros::Rate loop_rate(10);
    ros::init(argc, argv, "state_machine");
    RobotState = init;
    RobotStopped = False;
    wallClose = False;

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("laserScannerFront", 10, scannerFrontCallback);

    Server stopRobotServer(n, "stop_robot", boost::bind(&executeStopRobot, _1, &server), false);


    while(ros::ok())
    {
        RobotStateLastLoop = RobotState;

        if(wallClose == True)
        {
            stopRobotServer.start();
        }

        if(RobotStopped == True)
        {
            ros::shutdown();
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void scannerFrontCallback(const std_msgs::Bool::ConstPtr& wallClose)
{
    ((wallClose == True) ? (WallDetectedFront = True) : (WallDetectedFront = False));
}

void executeStopRobot(const jhonny5::MoveOrderGoalConstPtr& goal, Server* as)  // Note: "Action" is not appended to DoDishes here
{
    // Do lots of awesome groundbreaking robot stuff here
    as->setSucceeded();
    RobotStopped = True;
}