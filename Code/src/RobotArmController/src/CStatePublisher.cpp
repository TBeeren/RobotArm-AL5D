#include "CStatePublisher.h"
#include "RobotArmController/State.h"

/*static*/ CStatePublisher* CStatePublisher::m_pInstance = nullptr;

CStatePublisher::CStatePublisher()
: m_nodeHandle()
{
    m_statePublisher = m_nodeHandle.advertise<RobotArmController::State>("/RobotArmController/BehaviouralState",1000);
    m_protocolStatePublisher = m_nodeHandle.advertise<RobotArmController::State>("/RobotArmController/ProtocolState",1000);
}

CStatePublisher::~CStatePublisher()
{
    delete m_pInstance;
}

/*static*/ CStatePublisher* CStatePublisher::GetInstance()
{
    if (m_pInstance == nullptr)
    {
        m_pInstance = new CStatePublisher;
    }
    return m_pInstance;
}

void CStatePublisher::PublishState(ePublishableStates state)
{

    RobotArmController::State stateMessage;
    switch (state)
    {
    case PUBLISH_IDLE:
        {
            stateMessage.State = "Idle";
            break;
        }
    case PUBLISH_MOVE:
        {
            stateMessage.State = "Move";
            break;
        }
    case PUBLISH_CALIBRATE:
        {
            stateMessage.State = "Calibrate";
            break;
        }
    case PUBLISH_EMERGENCY_STOP:
        {
            stateMessage.State = "Emergency Stop";
            break;
        }
    
    default:
        {
            break;
        }
    }
    m_statePublisher.publish(stateMessage);

}

void CStatePublisher::PublishProtocolState(ePublishableStates state)
{

    RobotArmController::State stateMessage;
    switch (state)
    {
    case PUBLISH_IDLE:
        {
            stateMessage.State = "Idle";
            ROS_INFO("STATE: {IDLE}");
            break;
        }
    case PUBLISH_MOVE:
        {
            stateMessage.State = "Move";
            ROS_INFO("STATE: {MOVE}");
            break;
        }
    case PUBLISH_CALIBRATE:
        {
            stateMessage.State = "Move";
            ROS_INFO("STATE: {MOVE}");
            break;
        }
    case PUBLISH_EMERGENCY_STOP:
        {
            stateMessage.State = "Emergency Stop";
            ROS_INFO("STATE: {EMERGENCY_STOP}");
            break;
        } 
    default:
        {
            break;
        }
    }
    m_protocolStatePublisher.publish(stateMessage);

}