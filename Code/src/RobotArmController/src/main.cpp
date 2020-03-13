#include "CRobotContext.h"
#include "CStatePublisher.h"
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "RobotArmController");
    while(!ros::ok());
    ros::Time::init();
    ros::Duration duration(1);
    duration.sleep();
    CStatePublisher::GetInstance();
    CRobotContext context;
    context.Run();
    return 0;
}
