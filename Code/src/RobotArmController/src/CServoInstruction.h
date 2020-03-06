#ifndef CSERVOINSTRUCTION_H
#define CSERVOINSTRUCTION_H

#include <stdint.h>
#include "RobotHighLevel/CConfiguration.h"

class CServoInstruction
{
public:
    CServoInstruction(eServos targetServo, uint64_t position, uint64_t duration, uint64_t speed);
    ~CServoInstruction();

    eServos GetTargetServo();
    uint64_t GetPosition();
    uint64_t GetDuration();
    uint64_t GetSpeed();
private:
    eServos m_targetServo;
    uint64_t m_position;
    uint64_t m_duration;
    uint64_t m_speed;
};

#endif /*CSERVOINSTRUCTION_H*/