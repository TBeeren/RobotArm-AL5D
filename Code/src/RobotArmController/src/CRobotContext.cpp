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
        if(!m_preemptiveEventQueue.empty())
        {
            CEvent event(m_preemptiveEventQueue.front());
            m_preemptiveEventQueue.pop();
            m_spCurrentState->HandleEvent(event, *this);
        }
        else if(!m_queuedEventQueue.empty())
        {
            CEvent event(m_queuedEventQueue.front());
            m_queuedEventQueue.pop();
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
    std::vector<std::shared_ptr<CServoInstruction>> servoInstructions;
    for(const auto& instruction : moveMsg->instruction)
    {
        servoInstructions.emplace_back(std::make_shared<CServoInstruction>(static_cast<eServos>(instruction.targetServo), instruction.position, instruction.duration, instruction.speed));
    }

    CEvent event(MOVE, moveMsg->preemptive, servoInstructions);
    if(event.IsPreemptive())
    {
        m_preemptiveEventQueue.push(event);
    }
    else
    {
        m_queuedEventQueue.push(event);
    }
    
}

void CRobotContext::CalibrateCallback(const RobotArmController::Move::ConstPtr& calibrateMsg)
{
    std::vector<std::shared_ptr<CServoInstruction>> servoInstructions;
    for(auto instruction : calibrateMsg->instruction)
    {
        servoInstructions.emplace_back(std::make_shared<CServoInstruction>(static_cast<eServos>(instruction.targetServo), instruction.position, instruction.duration, instruction.speed));
    }
    CEvent event(MOVE, calibrateMsg->preemptive, servoInstructions);
    m_queuedEventQueue.push(event);
}

void CRobotContext::EmergencyStopCallback(const RobotArmController::EmergencyStop::ConstPtr& stopMsg)
{
    std::shared_ptr<IRobotStates> stopState = std::make_shared<CStopState>();
    SetState(stopState);
}

/* twee queues eerst de preemptive queue afhandelen
queues vullen
timer voor de gequeuede functies timer is gelijk aan hoogste duration plus timingdiagram tijd */

void CRobotContext::ProgrammedPositionCallback(const RobotArmController::ProgrammedPosition::ConstPtr& programmedPositionMsg)
{
    switch (m_spConfiguration->StringToProgrammedPosition(programmedPositionMsg->programmedPosition))
    {
    case PARK:
    {
        std::cout<<"TODO: IMPLEMENT PARK"<<std::endl;
        break;
    }
    case READY:
    {
        std::cout<<"TODO: IMPLEMENT READY"<<std::endl;
        break;
    }
    case STRAIGHT:
    {
        std::cout<<"TODO: IMPLEMENT STRAIGHT"<<std::endl;
        break;
    }
    default:
        break;
    } 
}