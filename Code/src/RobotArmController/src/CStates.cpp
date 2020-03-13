#include "CStates.h"
#include "CRobotContext.h"
#include "CStatePublisher.h"
#include "RobotHighLevel/CCalibration.h"
#include "RobotHighLevel/CMove.h"
#include "RobotLowLevel/IExecuteCommand.h"
#include <iostream>

CIdleState::CIdleState(CEvent& rEvent, std::shared_ptr<CConfiguration> spConfiguration)
: IRobotStates()
, m_Event(rEvent)
, m_spConfiguration(spConfiguration)
{
}

CIdleState::~CIdleState()
{
}

void CIdleState::Entry()
{
    CStatePublisher::GetInstance()->PublishState(ePublishableStates::PUBLISH_IDLE);
    CStatePublisher::GetInstance()->PublishProtocolState(ePublishableStates::PUBLISH_IDLE);
}
void CIdleState::Do()
{

}
void CIdleState::Exit()
{

}

void CIdleState::HandleEvent(CEvent& rEvent, CRobotContext& rContext)
{
    switch (rEvent.GetEventType())
    {
    case MOVE:
    {
        std::shared_ptr<IRobotStates> newState = std::make_shared<CMoveState>(rEvent, m_spConfiguration);
        rContext.SetState(newState);
        break;
    }
    case EMERGENCY_STOP:
    {
        std::shared_ptr<IRobotStates> newState = std::make_shared<CStopState>(rEvent, m_spConfiguration);
        rContext.SetState(newState);
        break;
    }
    case CALIBRATE:
    {
        std::shared_ptr<IRobotStates> newState = std::make_shared<CCalibrateState>(rEvent, m_spConfiguration);
        rContext.SetState(newState);
        break;
    }
    default:
    {
        break;
    }
    }
}

CCalibrateState::CCalibrateState(CEvent& rEvent, std::shared_ptr<CConfiguration> spConfiguration)
: IRobotStates()
, m_Event(rEvent)
, m_spConfiguration(spConfiguration)
{
}

CCalibrateState::~CCalibrateState()
{
}

void CCalibrateState::Entry()
{
    CStatePublisher::GetInstance()->PublishState(ePublishableStates::PUBLISH_CALIBRATE);
    CStatePublisher::GetInstance()->PublishProtocolState(ePublishableStates::PUBLISH_CALIBRATE);
    CCalibration calibration(m_spConfiguration);
    calibration.Execute(eCommand::CALIBRATE_COMMAND, m_Event.GetServoInstructions());
}
void CCalibrateState::Do()
{
}
void CCalibrateState::Exit()
{
    
}

void CCalibrateState::HandleEvent(CEvent& rEvent, CRobotContext& rContext)
{
    switch (rEvent.GetEventType())
    {
    case IDLE:
    {
        std::shared_ptr<IRobotStates> newState = std::make_shared<CIdleState>(rEvent, m_spConfiguration);
        rContext.SetState(newState);
        break;
    }
    case MOVE:
    {
        std::shared_ptr<IRobotStates> newState = std::make_shared<CMoveState>(rEvent, m_spConfiguration);
        rContext.SetState(newState);
        break;
    }
    case EMERGENCY_STOP:
    {
        std::shared_ptr<IRobotStates> newState = std::make_shared<CStopState>(rEvent, m_spConfiguration);
        rContext.SetState(newState);
        break;
    }
    case CALIBRATE:
    {
        std::shared_ptr<IRobotStates> newState = std::make_shared<CCalibrateState>(rEvent, m_spConfiguration);
        rContext.SetState(newState);
        break;
    }
    default:
    {
        std::cout<<"State transition not implemented"<<std::endl;
        break;
    }
    }
}

CMoveState::CMoveState(CEvent& rEvent, std::shared_ptr<CConfiguration> spConfiguration)
: IRobotStates()
, m_Event(rEvent)
, m_spConfiguration(spConfiguration)
{
}

CMoveState::~CMoveState()
{
}

void CMoveState::Entry()
{
    CStatePublisher::GetInstance()->PublishState(ePublishableStates::PUBLISH_MOVE);   
    CStatePublisher::GetInstance()->PublishProtocolState(ePublishableStates::PUBLISH_MOVE);   
    CMove move(m_spConfiguration);
    move.Execute(MOVE_COMMAND, m_Event.GetServoInstructions());
}
void CMoveState::Do()
{
}
void CMoveState::Exit()
{
    
}

void CMoveState::HandleEvent(CEvent& rEvent, CRobotContext& rContext)
{
    switch (rEvent.GetEventType())
    {
    case IDLE:
    {
        std::shared_ptr<IRobotStates> newState = std::make_shared<CIdleState>(rEvent, m_spConfiguration);
        rContext.SetState(newState);
        break;
    }
    case MOVE:
    {
        std::shared_ptr<IRobotStates> newState = std::make_shared<CMoveState>(rEvent, m_spConfiguration);
        rContext.SetState(newState);
        break;
    }
    case EMERGENCY_STOP:
    {
        std::shared_ptr<IRobotStates> newState = std::make_shared<CStopState>(rEvent, m_spConfiguration);
        rContext.SetState(newState);
        break;
    }
    case CALIBRATE:
    {
        std::shared_ptr<IRobotStates> newState = std::make_shared<CCalibrateState>(rEvent, m_spConfiguration);
        rContext.SetState(newState);
        break;
    }
    default:
    {
        std::cout<<"State transition not implemented"<<std::endl;
        break;
    }
    }
}

CStopState::CStopState(CEvent& rEvent, std::shared_ptr<CConfiguration> spConfiguration)
: IRobotStates()
, m_Event(rEvent)
, m_spConfiguration(spConfiguration)
{
}

CStopState::~CStopState()
{
}

void CStopState::Entry()
{
    CStatePublisher::GetInstance()->PublishState(ePublishableStates::PUBLISH_EMERGENCY_STOP); 
    CStatePublisher::GetInstance()->PublishProtocolState(ePublishableStates::PUBLISH_EMERGENCY_STOP); 
    CMove move(m_spConfiguration);
    move.Execute(STOP_COMMAND, m_Event.GetServoInstructions());
}
void CStopState::Do()
{
}
void CStopState::Exit()
{
    
}

void CStopState::HandleEvent(CEvent& rEvent, CRobotContext& rContext)
{
    std::cout<<"NO STATES WILL BE HANDLED DURING EMERGENCY STOP, RESTART APPLICATION"<<std::endl;
}
