#include "CStates.h"

CIdleState::CIdleState(CEvent& rEvent, std::shared_ptr<CConfiguration> spConfiguration)
: IRobotStates()
, m_rEvent(rEvent)
, m_spConfiguration(spConfiguration)
{
}

CIdleState::~CIdleState()
{
}

void CIdleState::Entry()
{

}
void CIdleState::Do()
{

}
void CIdleState::Exit()
{

}

void CIdleState::HandleEvent(CEvent& rEvent, CRobotContext& rContext)
{

}

CCalibrateState::CCalibrateState(CEvent& rEvent, std::shared_ptr<CConfiguration> spConfiguration)
: IRobotStates()
, m_rEvent(rEvent)
, m_spConfiguration(spConfiguration)
{
}

CCalibrateState::~CCalibrateState()
{
}

void CCalibrateState::Entry()
{
    
}
void CCalibrateState::Do()
{
    
}
void CCalibrateState::Exit()
{
    
}

void CCalibrateState::HandleEvent(CEvent& rEvent, CRobotContext& rContext)
{

}

CMoveState::CMoveState(CEvent& rEvent, std::shared_ptr<CConfiguration> spConfiguration)
: IRobotStates()
, m_rEvent(rEvent)
, m_spConfiguration(spConfiguration)
{
}

CMoveState::~CMoveState()
{
}

void CMoveState::Entry()
{
    
}
void CMoveState::Do()
{
    
}
void CMoveState::Exit()
{
    
}

void CMoveState::HandleEvent(CEvent& rEvent, CRobotContext& rContext)
{

}

CStopState::CStopState(CEvent& rEvent, std::shared_ptr<CConfiguration> spConfiguration)
: IRobotStates()
, m_rEvent(rEvent)
, m_spConfiguration(spConfiguration)
{
}

CStopState::~CStopState()
{
}

void CStopState::Entry()
{
    
}
void CStopState::Do()
{
    
}
void CStopState::Exit()
{
    
}

void CStopState::HandleEvent(CEvent& rEvent, CRobotContext& rContext)
{

}
