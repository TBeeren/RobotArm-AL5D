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

}