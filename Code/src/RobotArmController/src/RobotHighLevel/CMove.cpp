/**
 * @file CMove.cpp
 * @author Tim Beeren (T.Beeren1@student.han.nl)
 * @brief 
 * @version 0.1
 * @date 06-03-2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "CMove.h"

#include "../CServoInstruction.h"
#include "../RobotLowLevel/CCommandAL5D.h"
#include <assert.h>

namespace
{
    constexpr const uint16_t MINIMAL_PULSE_WIDTH =  500;
    constexpr const uint16_t MAXIMAL_PULSE_WIDTH =  2500;

    constexpr const uint8_t MAXIMAL_DEGREES_BASE = 180;
    constexpr const uint8_t MAXIMAL_DEGREES_SHOULDER = 120;
    constexpr const uint8_t MAXIMAL_DEGREES_ELBOW = 135;
    constexpr const uint8_t MAXIMAL_DEGREES_WRIST = 180;
    constexpr const uint8_t MAXIMAL_DEGREES_WRIST_ROTATE = 180;
}

CMove::CMove(std::shared_ptr<CConfiguration> spConfiguration)
: m_spConfiguration(spConfiguration)
, m_spExecuteCommand(std::make_shared<CCommandAL5D>())
{
}

CMove::~CMove()
{
}

uint16_t CMove::DegreesToPWM(eServos servo, int16_t degrees)
{   
    assert(servo != eServos::UNKNOWN_SERVO);
    uint16_t PWM;

    switch (servo)
    {
        case eServos::BASE:
        {
            PWM = ((static_cast<uint16_t>(MAXIMAL_PULSE_WIDTH) / MAXIMAL_DEGREES_BASE) * degrees);
            break;
        }
        case eServos::SHOULDER:
        {
            PWM = ((static_cast<uint16_t>(MAXIMAL_PULSE_WIDTH) / MAXIMAL_DEGREES_SHOULDER) * degrees);
            break;
        }
        case eServos::ELBOW:
        {
            PWM = ((static_cast<uint16_t>(MAXIMAL_PULSE_WIDTH) / MAXIMAL_DEGREES_ELBOW) * degrees);
            break;
        }
        case eServos::WRIST:
        {
            PWM = ((static_cast<uint16_t>(MAXIMAL_PULSE_WIDTH) / MAXIMAL_DEGREES_WRIST) * degrees);
            break;
        }
        case eServos::GRIPPER:
        {
            PWM = ((static_cast<uint16_t>(MAXIMAL_PULSE_WIDTH) / MAXIMAL_DEGREES_WRIST) * degrees);
            break;
        }
        case eServos::WRIST_ROTATE:
        {
            PWM = ((static_cast<uint16_t>(MAXIMAL_PULSE_WIDTH) / MAXIMAL_DEGREES_WRIST_ROTATE) * degrees);
            break;
        }
        default:
        {
            std::cout<< "Servo is unknown! Please check your message on syntax" << std::endl;
            break;
        }
    }

    return PWM;
}

bool CMove::IsInstructionValid(std::shared_ptr<CServoInstruction> rServoInstruction)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    if(rServoInstruction->GetTargetServo() >= eServos::UNKNOWN_SERVO)
    {
        return false;
    }
    if(rServoInstruction->GetSpeed() > 65535 || rServoInstruction->GetSpeed() == 0)
    {
        return false;
    }
    return true;
}

void CMove::Execute(eCommand eCommand, std::vector<std::shared_ptr<CServoInstruction>> rServoInstructions)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    assert(rServoInstructions.size() != 0);

    for(const auto& rInstruction : rServoInstructions)
    {
        m_spExecuteCommand->AppendInstruction(
            eCommand, 
            rInstruction->GetTargetServo(), 
            DegreesToPWM(rInstruction->GetTargetServo(), rInstruction->GetPosition()),
            rInstruction->GetSpeed(), 
            rInstruction->GetDuration()
        );

        if(!IsInstructionValid(rInstruction))
        {
            m_spExecuteCommand->ClearLists();
            //throw "Wrong Instruction! Please provide valid syntax";
            break; 
        }
    }

    m_spExecuteCommand->Execute();
}