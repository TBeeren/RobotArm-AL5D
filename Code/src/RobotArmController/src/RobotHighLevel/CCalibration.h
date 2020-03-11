/**
 * @file CCalibration.h
 * @author Tim Beeren (T.Beeren1@student.han.nl)
 * @brief 
 * @version 0.1
 * @date 06-03-2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef CCALIBRATION_H
#define CCALIBRATION_H

#include <memory>
#include <vector>

#include "CConfiguration.h"
#include "../RobotLowLevel/IExecuteCommand.h"

// Forward Declaration
class CServoInstruction;
class CMove;

class CCalibration
{
public:
    explicit CCalibration(std::shared_ptr<CConfiguration> spConfiguration);
    virtual ~CCalibration();

    void Execute(eCommand eCommand, std::vector<std::shared_ptr<CServoInstruction>> rServoIntructions);

private:
    bool WriteConfig(eServos eServo, uint16_t value);
    std::shared_ptr<CConfiguration> m_spConfiguration;
    std::shared_ptr<CMove> m_spMove;
    
};

#endif /*CCALIBRATION_H*/
