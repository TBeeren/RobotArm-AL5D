/**
 * @file CStatePublisher.h
 * @author Evren Kilic (ET.Kilic@student.han.nl)
 * @brief 
 * @version 0.1
 * @date 05-03-2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef CSTATEPUBLISHER_H
#define CSTATEPUBLISHER_H

#include <ros/ros.h>

enum ePublishableStates
{
    PUBLISH_IDLE,
    PUBLISH_MOVE,
    PUBLISH_CALIBRATE,
    PUBLISH_EMERGENCY_STOP,
};

class CStatePublisher
{
public:
    ~CStatePublisher();

    static CStatePublisher* GetInstance();
    void PublishState(ePublishableStates state);

private:
    CStatePublisher();
    static CStatePublisher* m_pInstance;
    ros::NodeHandle m_nodeHandle;
    ros::Publisher m_publisher;
};

#endif /*CSTATEPUBLISHER_H*/