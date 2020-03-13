/**
 * @file CServoInstruction.h
 * @author Evren Kilic (ET.Kilic@student.han.nl)
 * @brief CServoinstruction is a class that holds the instructions for a specfic servo within the robotarm.
 * @version 0.1
 * @date 11-03-2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef CSERVOINSTRUCTION_H
#define CSERVOINSTRUCTION_H

#include <stdint.h>
#include "RobotHighLevel/CConfiguration.h"

class CServoInstruction
{
public:
    CServoInstruction(eServos targetServo, int64_t position, int64_t duration, int64_t speed);
    ~CServoInstruction();

    eServos GetTargetServo();
    int64_t GetPosition();
    int64_t GetDuration();
    int64_t GetSpeed();
private:
    eServos m_targetServo;
    int64_t m_position;
    int64_t m_duration;
    int64_t m_speed;
};

#endif /*CSERVOINSTRUCTION_H*/