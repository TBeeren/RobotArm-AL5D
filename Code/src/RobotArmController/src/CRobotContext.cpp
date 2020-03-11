#include "CRobotContext.h"
#include "CServoInstruction.h"
#include "CStates.h"
#include "RobotHighLevel/CConfiguration.h"
#include "RobotArmController/ServoCommand.h"

namespace
{
    constexpr const std::chrono::duration<double> WORST_CASE_LATENCY_MILLIS = std::chrono::milliseconds(2300);
}

CRobotContext::CRobotContext()
: m_spCurrentState(nullptr)
, m_spConfiguration(std::make_shared<CConfiguration>())
, m_nodeHandle()
, m_moveSubcriber(m_nodeHandle.subscribe("/RobotArmController/Move",1000, &CRobotContext::MoveCallback, this))
, m_calibrateSubscriber(m_nodeHandle.subscribe("/RobotArmController/Calibrate",1000, &CRobotContext::CalibrateCallback, this))
, m_emergencyStopSubcriber(m_nodeHandle.subscribe("/RobotArmController/EmergencyStop", 1000, &CRobotContext::EmergencyStopCallback, this))
, m_programmedPositionSubcriber(m_nodeHandle.subscribe("/RobotArmController/ProgrammedPosition", 1000, &CRobotContext::ProgrammedPositionCallback, this))
{
    CEvent event(IDLE);
    std::shared_ptr<IRobotStates> spStartState = std::make_shared<CIdleState>(event, m_spConfiguration);
    SetState(spStartState);
}

CRobotContext::~CRobotContext()
{
}

void CRobotContext::Run()
{
    auto currentTime = std::chrono::system_clock::now();
    auto deadline = currentTime + WORST_CASE_LATENCY_MILLIS;    

    do
    {
        if(!m_preemptiveEventQueue.empty())
        {
            CEvent event(m_preemptiveEventQueue.front());
            deadline = std::chrono::system_clock::now() + WORST_CASE_LATENCY_MILLIS + GetHighestDuration(event);
            m_preemptiveEventQueue.pop();
            m_spCurrentState->HandleEvent(event, *this);
        }
        else if(!m_queuedEventQueue.empty() && deadline <= std::chrono::system_clock::now())
        {
            CEvent event(m_queuedEventQueue.front());
            deadline = std::chrono::system_clock::now() + WORST_CASE_LATENCY_MILLIS + GetHighestDuration(event);
            m_queuedEventQueue.pop();
            m_spCurrentState->HandleEvent(event, *this);
        }
        ros::spinOnce();
    }while(ros::ok());
}

void CRobotContext::SetState(std::shared_ptr<IRobotStates> sp_state)
{
    if(m_spCurrentState != nullptr)
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
    CEvent event(CALIBRATE, calibrateMsg->preemptive, servoInstructions);
    if(event.IsPreemptive())
    {
        m_preemptiveEventQueue.push(event);
    }
    else
    {
        m_queuedEventQueue.push(event);
    }
}

void CRobotContext::EmergencyStopCallback(const RobotArmController::EmergencyStop::ConstPtr& stopMsg)
{
    CEvent event(EMERGENCY_STOP);
    std::shared_ptr<IRobotStates> stopState = std::make_shared<CStopState>(event, m_spConfiguration);
    SetState(stopState);
}

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

std::chrono::duration<double> CRobotContext::GetHighestDuration(CEvent& rEvent)
{
    uint64_t highestDuration = 0;
    for(std::shared_ptr<CServoInstruction> instruction : rEvent.GetServoInstructions())
    {
        if(instruction->GetDuration() >  highestDuration)
        {
            highestDuration = instruction->GetDuration();
        }
    }
    return std::chrono::milliseconds(highestDuration);
}