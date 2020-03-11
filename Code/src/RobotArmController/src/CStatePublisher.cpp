#include "CStatePublisher.h"
#include "RobotArmController/State.h"

/*static*/ CStatePublisher* CStatePublisher::m_pInstance = nullptr;

CStatePublisher::CStatePublisher()
: m_nodeHandle()
{
    m_publisher = m_nodeHandle.advertise<RobotArmController::State>("/RobotArmController/State",1000);
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
    m_publisher.publish(stateMessage);

}