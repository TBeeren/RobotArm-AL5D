#include "CStatePublisher.h"
#include "RobotArmController/State.h"

CStatePublisher::CStatePublisher()
: m_nodeHandle()
, m_publisher(m_nodeHandle.advertise<RobotArmController::State>("\\MotionController\\State",1000))
{
}

CStatePublisher::~CStatePublisher()
{
}

CStatePublisher CStatePublisher::GetInstance()
{
    static CStatePublisher statePublisher;
    return statePublisher;
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