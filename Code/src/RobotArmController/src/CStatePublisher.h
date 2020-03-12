/**
 * @file CStatePublisher.h
 * @author Evren Kilic (ET.Kilic@student.han.nl)
 * @brief CStatePublisher is a singleton class that can be used to publish a state on the /RobotArmController/State ros topic
 * @version 0.1
 * @date 05-03-2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef CSTATEPUBLISHER_H
#define CSTATEPUBLISHER_H

#include <ros/ros.h>

/**
 * @brief The enum of states that the state publisher can publish
 * 
 */
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

    /**
     * @brief Returns the single instance of the CStatePublisher class
     * 
     * @return CStatePublisher* A pointer to the singleton statepublisher
     */
    static CStatePublisher* GetInstance();
    /**
     * @brief Publishes the state on the /RobotArmController/BehaviouralState ros topic
     * 
     * @param state the state to publish on the before mentioned ros topic
     */
    void PublishState(ePublishableStates state);
    /**
     * @brief Publishes the state on the /RobotArmController/ProtocolState ros topic
     * 
     * @param state the state to publish on the before mentioned ros topic
     */
    void PublishProtocolState(ePublishableStates state);

private:
    /**
     * @brief Private constructior for the state publisher as it is a singleton.
     * 
     */
    CStatePublisher();
    static CStatePublisher* m_pInstance;
    ros::NodeHandle m_nodeHandle;
    ros::Publisher m_statePublisher;
    ros::Publisher m_protocolStatePublisher;
};

#endif /*CSTATEPUBLISHER_H*/