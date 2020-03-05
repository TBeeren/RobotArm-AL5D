#ifndef CSTATEPUBLISHER_H
#define CSTATEPUBLISHER_H

#include <ros/ros.h>

class CStatePublisher
{
public:
    ~CStatePublisher();

    CStatePublisher GetInstance();

private:
    CStatePublisher();
    ros::NodeHandle m_nodeHandle;
    ros::Publisher m_publisher;
};

#endif /*CSTATEPUBLISHER_H*/