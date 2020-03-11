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
    constexpr const float MINIMAL_PWM_VALUE = 500.0;
    constexpr const float MAXIMUM_PWM_VALUE = 2500.0;

    constexpr const int16_t MINIMAL_DEGREES_BASE = -90;
    constexpr const int16_t MINIMAL_DEGREES_SHOULDER = -30;
    constexpr const int16_t MINIMAL_DEGREES_ELBOW = 0;
    constexpr const int16_t MINIMAL_DEGREES_WRIST = -90;
    constexpr const int16_t MINIMAL_DEGREES_WRIST_ROTATE = -90;

    constexpr const int16_t MAXIMAL_DEGREES_BASE = 90;
    constexpr const int16_t MAXIMAL_DEGREES_SHOULDER = 90;
    constexpr const int16_t MAXIMAL_DEGREES_ELBOW = 135;
    constexpr const int16_t MAXIMAL_DEGREES_WRIST = 90;
    constexpr const int16_t MAXIMAL_DEGREES_WRIST_ROTATE = 90;

    constexpr const int16_t TOTAL_DEGREES_BASE = 180;
    constexpr const int16_t TOTAL_DEGREES_SHOULDER = 120;
    constexpr const int16_t TOTAL_DEGREES_ELBOW = 135;
    constexpr const int16_t TOTAL_DEGREES_WRIST = 180;
    constexpr const int16_t TOTAL_DEGREES_WRIST_ROTATE = 180;
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
            PWM = ((static_cast<uint16_t>((m_spConfiguration->GetMaxPWM(servo) - m_spConfiguration->GetMinPWM(servo)) / TOTAL_DEGREES_BASE) * degrees) 
            + (m_spConfiguration->GetMaxPWM(servo) + m_spConfiguration->GetMinPWM(servo))/2);
            break;
        }
        case eServos::SHOULDER:
        {
            PWM = ((static_cast<uint16_t>((m_spConfiguration->GetMaxPWM(servo) - m_spConfiguration->GetMinPWM(servo)) / TOTAL_DEGREES_SHOULDER) * degrees) 
            + (m_spConfiguration->GetMaxPWM(servo) + m_spConfiguration->GetMinPWM(servo))/2);
            break;
        }
        case eServos::ELBOW:
        {
            PWM = ((static_cast<uint16_t>((m_spConfiguration->GetMaxPWM(servo) - m_spConfiguration->GetMinPWM(servo)) / TOTAL_DEGREES_ELBOW) * degrees) 
            + (m_spConfiguration->GetMaxPWM(servo) + m_spConfiguration->GetMinPWM(servo))/2);
            break;
        }
        case eServos::WRIST:
        {
            PWM = ((static_cast<uint16_t>((m_spConfiguration->GetMaxPWM(servo) - m_spConfiguration->GetMinPWM(servo)) / TOTAL_DEGREES_WRIST) * degrees) 
            + (m_spConfiguration->GetMaxPWM(servo) + m_spConfiguration->GetMinPWM(servo))/2);
           break;
        }
        case eServos::GRIPPER:
        {
            PWM = ((static_cast<uint16_t>((m_spConfiguration->GetMaxPWM(servo) - m_spConfiguration->GetMinPWM(servo)) / TOTAL_DEGREES_WRIST_ROTATE) * degrees) 
            + (m_spConfiguration->GetMaxPWM(servo) + m_spConfiguration->GetMinPWM(servo))/2);
           break;
        }
        case eServos::WRIST_ROTATE:
        {
            PWM = ((static_cast<uint16_t>((m_spConfiguration->GetMaxPWM(servo) - m_spConfiguration->GetMinPWM(servo)) / TOTAL_DEGREES_WRIST_ROTATE) * degrees) 
            + (m_spConfiguration->GetMaxPWM(servo) + m_spConfiguration->GetMinPWM(servo))/2);
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

uint16_t CMove::CalibrationDegreesToPwm(int16_t degrees)
{
    std::cout<<"Degrees: " << degrees<< std::endl;
    std::cout<<"PWM:  " << (static_cast<uint16_t>((MAXIMUM_PWM_VALUE - MINIMAL_PWM_VALUE) / 150.0) * degrees) + ((MINIMAL_PWM_VALUE + MAXIMUM_PWM_VALUE)/2)<< std::endl;
    return (static_cast<uint16_t>((MAXIMUM_PWM_VALUE - MINIMAL_PWM_VALUE) / 150.0) * degrees) + ((MINIMAL_PWM_VALUE + MAXIMUM_PWM_VALUE)/2);

}

bool CMove::IsInstructionValid(std::shared_ptr<CServoInstruction> rServoInstruction)
{
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
    bool success = true;

    for(const auto& rInstruction : rServoInstructions)
    {
        if(eCommand == eCommand::CALIBRATE_COMMAND)
        {
            m_spExecuteCommand->AppendInstruction(
                eCommand, 
                rInstruction->GetTargetServo(), 
                CalibrationDegreesToPwm(rInstruction->GetPosition()),
                rInstruction->GetSpeed(), 
                rInstruction->GetDuration()
            );
        }
        else
        {
            m_spExecuteCommand->AppendInstruction(
                eCommand, 
                rInstruction->GetTargetServo(), 
                DegreesToPWM(rInstruction->GetTargetServo(), rInstruction->GetPosition()),
                rInstruction->GetSpeed(), 
                rInstruction->GetDuration()
            );
        }

        if(!IsInstructionValid(rInstruction))
        {
            m_spExecuteCommand->ClearLists();
            std::cout<< " [Warning]-> Wrong Instruction! Please provide valid syntax"<< std::endl;
            success = false;
            break; 
        }
    }

    if(success)
    {
        m_spExecuteCommand->Execute();
    }
}