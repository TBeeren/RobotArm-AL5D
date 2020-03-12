/**
 * @file CMove.h
 * @author Tim Beeren (T.Beeren1@student.han.nl)
 * @brief The CMove class offers functions to calculate degrees to PWM and to then send a command to the robotarm that will make it move.
 * @version 0.1
 * @date 12-03-2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef CMOVE_H
#define CMOVE_H

#include <vector>
#include <memory>
#include "../RobotLowLevel/IExecuteCommand.h"
#include "CConfiguration.h"

// Forward Declaration
class CServoInstruction;
class CCommandAL5D;

class CMove
{
public:
    CMove(std::shared_ptr<CConfiguration> spConfiguration);
    ~CMove();

    /**
     * @brief Executes servo commands and ultimately moves the robotarm when instructions are valid
     * 
     * @param eCommand the type of command to be executed by the robotarm
     * @param rServoInstructions a list of instructions for the different servos
     */
    void Execute(eCommand eCommand, std::vector<std::shared_ptr<CServoInstruction>> rServoInstructions);
    /**
     * @brief Executes servo commands and ultimately moves the robotarm when instructions are valid. Using this function will set the values in the robotarms configuration when they surpass current boundries.
     * 
     * @param eCommand the type of command to be executed by the robotarm(usually CALIBRATE_COMMAND)
     * @param rServoInstructions a list of instructions for the different servos
     */
    void ExecuteCalibrate(eCommand eCommand, std::vector<std::shared_ptr<CServoInstruction>> rServoInstructions);
    /**
     * @brief Checks the validity of servo instructions based on the minimal angle and maximum angle of the servo's
     * 
     * @param rServoInstruction the servoinstruction which will be tested for it's validity
     * @return true Returns true if the instruction is valid
     * @return false Returns false if the instruction is invalid
     */
    bool IsInstructionValid(std::shared_ptr<CServoInstruction> rServoInstruction);
    /**
     * @brief Calculates the degrees, commonly stored in servoinstructions, to PWM for a specific servo based on its ranges.
     * 
     * @param servo the servo whose minimum and maximum values will be used in the calculation
     * @param degrees the degrees which will de rewritten into PWM
     * @return uint16_t returns the amount of PWM that matches the degrees
     */
    uint16_t DegreesToPWM(eServos servo, int16_t degrees);
    /**
     * @brief Calculates the degrees, commonly storedd in servoinstructions, to PWM for a specific servo based on its ranges. 
     * This function is used when commanding the robotarm during calibration. For this calculation a standardised set of upper and lower limits is used
     * 
     * @param degrees the degrees to be translated to PWM
     * @return uint16_t the PWM which matches the degrees
     */
    uint16_t CalibrationDegreesToPwm(int16_t degrees);
    /**
     * @brief This function gets the upper and lower limit for a servo which has an instruction for it.
     * 
     * @param instruction the instruction for a specific servo
     * @return std::vector<int> the lower limit of the servo followed by the upper limit of the servo.
     */
    std::vector<int> GetMinAndMaxValue(std::shared_ptr<CServoInstruction> instruction);

private:
    std::shared_ptr<CConfiguration> m_spConfiguration;
    std::shared_ptr<IExecuteCommand> m_spExecuteCommand;
};

#endif /*CMOVE_H*/