#include "CServoInstruction.h"

CServoInstruction::CServoInstruction(eServos targetServo, uint64_t position, uint64_t duration, uint64_t speed)
: m_targetServo(targetServo)
, m_position(position)
, m_duration(duration)
, m_speed(speed)
{
}

CServoInstruction::~CServoInstruction()
{
}

eServos CServoInstruction::GetTargetServo()
{
    return m_targetServo;
}

uint64_t CServoInstruction::GetPosition()
{
    return m_position;
}

uint64_t CServoInstruction::GetDuration()
{
    return m_duration;
}

uint64_t CServoInstruction::GetSpeed()
{
    return m_speed;
}