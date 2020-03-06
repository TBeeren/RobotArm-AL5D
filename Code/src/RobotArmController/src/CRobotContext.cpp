#include "CRobotContext.h"
#include "CServoInstruction.h"
#include "CStates.h"
#include "RobotHighLevel/CConfiguration.h"
#include "RobotArmController/ServoCommand.h"

CRobotContext::CRobotContext()
: m_spCurrentState(nullptr)
, m_spConfiguration(std::make_shared<CConfiguration>())
, m_nodeHandle()
, m_moveSubcriber(m_nodeHandle.subscribe("\\RobotArmController\\Move",1000, &CRobotContext::MoveCallback, this))
, m_calibrateSubscriber(m_nodeHandle.subscribe("\\RobotArmController\\Calibrate",1000, &CRobotContext::CalibrateCallback, this))
, m_emergencyStopSubcriber(m_nodeHandle.subscribe("\\RobotArmController\\EmergencyStop", 1000, &CRobotContext::EmergencyStopCallback, this))
, m_programmedPositionSubcriber(m_nodeHandle.subscribe("\\RobotArmController\\ProgrammedPosition", 1000, &CRobotContext::ProgrammedPositionCallback, this))
{
    CEvent event(IDLE);
    std::shared_ptr<IRobotStates> spStartState = std::make_shared<CIdleState>(event, m_spConfiguration);
    SetState(spStartState);
}

CRobotContext::~CRobotContext()
{
}

void CRobotContext::Init()
{
    CEvent event(IDLE);
    std::shared_ptr<IRobotStates> spStartState = std::make_shared<CIdleState>(event, m_spConfiguration);
    SetState(spStartState);
}

void CRobotContext::Run()
{
    do
    {
        while(!m_eventQueue.empty())
        {
            CEvent event(m_eventQueue.front());
            m_eventQueue.pop();
            m_spCurrentState->HandleEvent(event, *this);
        }
        ros::spinOnce();
    }while(ros::ok);
}

void CRobotContext::SetState(std::shared_ptr<IRobotStates> sp_state)
{
    if(!m_spCurrentState)
    {
        m_spCurrentState->Exit();
    }
    m_spCurrentState = sp_state;
    m_spCurrentState->Entry();
    m_spCurrentState->Do();
}

void CRobotContext::MoveCallback(const RobotArmController::Move::ConstPtr& moveMsg)
{
    std::vector<CServoInstruction> servoInstructions;
    for(auto instruction : moveMsg->instruction)
    {
        servoInstructions.emplace_back(static_cast<eServos>(instruction.targetServo), instruction.position, instruction.duration, instruction.speed);
    }
    CEvent event(MOVE, moveMsg->preemptive, servoInstructions);
    m_eventQueue.push(event);
}

void CRobotContext::CalibrateCallback(const RobotArmController::Move::ConstPtr& calibrateMsg)
{
    std::vector<CServoInstruction> servoInstructions;
    for(auto instruction : calibrateMsg->instruction)
    {
        servoInstructions.emplace_back(static_cast<eServos>(instruction.targetServo), instruction.position, instruction.duration, instruction.speed);
    }
    CEvent event(MOVE, calibrateMsg->preemptive, servoInstructions);
    m_eventQueue.push(event);
}

void CRobotContext::EmergencyStopCallback(const RobotArmController::EmergencyStop::ConstPtr& stopMsg)
{
    CEvent event(EMERGENCY_STOP);
    m_eventQueue.push(event);
}

void CRobotContext::ProgrammedPositionCallback(const RobotArmController::ProgrammedPosition::ConstPtr& programmedPositionMsg)
{

}