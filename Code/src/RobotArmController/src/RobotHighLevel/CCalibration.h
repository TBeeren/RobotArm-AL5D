/**
 * @file CCalibration.h
 * @author Tim Beeren (T.Beeren1@student.han.nl)
 * @brief The Calibration class is responsible for handling calibrations for the servo boundries
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

    /**
     * @brief Executes a list of servoinstructions
     * 
     * @param eCommand the type of command(usually a CALIBRATE_COMMAND when used within this class)
     * @param rServoIntructions the servoinstructions to have the robotarm execute
     */
    void Execute(eCommand eCommand, std::vector<std::shared_ptr<CServoInstruction>> rServoIntructions);

private:
    /**
     * @brief Writes the value that is used as a servoinstruction to the configuration class
     * 
     * @param eServo the servo for which the change in configuration implies
     * @param value The value that will be potentially written into the configuration for the servo
     * @return true Returns true if the value was successfully changed
     * @return false Returns false if the value was left unchanged 
     */
    bool WriteConfig(eServos eServo, uint16_t value);
    std::shared_ptr<CConfiguration> m_spConfiguration;
    std::shared_ptr<CMove> m_spMove;
    
};

#endif /*CCALIBRATION_H*/
