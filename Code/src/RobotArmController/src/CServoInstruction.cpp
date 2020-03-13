#include "CServoInstruction.h"

CServoInstruction::CServoInstruction(eServos targetServo, int64_t position, int64_t duration, int64_t speed)
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

int64_t CServoInstruction::GetPosition()
{
    return m_position;
}

int64_t CServoInstruction::GetDuration()
{
    return m_duration;
}

int64_t CServoInstruction::GetSpeed()
{
    return m_speed;
}