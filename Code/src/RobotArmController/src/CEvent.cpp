#include "CEvent.h"
#include "CServoInstruction.h"
#include <ros/ros.h>

CEvent::CEvent(eEventType eventType)
: m_eventType(eventType)
, m_preemptive(false)
{

}


CEvent::CEvent(eEventType eventType, bool preemptive, std::vector<std::shared_ptr<CServoInstruction>> servoInstructions)
: m_eventType(eventType)
, m_preemptive(preemptive)
, m_spServoInstructions(servoInstructions)
{
    switch(m_eventType)
    {
        case IDLE:
        {
            ROS_DEBUG("EVENT: {evIDLE}");
            break;
        }
        case MOVE:
        {
            ROS_DEBUG("EVENT: {evMOVE}");
            break;
        }
        case CALIBRATE:
        {
            ROS_DEBUG("EVENT: {evCALIBRATE}");
            break;
        }
        case EMERGENCY_STOP:
        {
            ROS_DEBUG("EVENT: {evEMERGENCY_STOP}");
            break;
        }
    }
}

CEvent::CEvent(const CEvent& event)
: m_eventType(event.m_eventType)
, m_preemptive(event.m_preemptive)
, m_spServoInstructions(event.m_spServoInstructions)
{

}

CEvent::~CEvent()
{
}

bool CEvent::IsPreemptive()
{
    return m_preemptive;
}

eEventType CEvent::GetEventType()
{
    return m_eventType;
}

std::vector<std::shared_ptr<CServoInstruction>> CEvent::GetServoInstructions()
{
    return m_spServoInstructions;
}