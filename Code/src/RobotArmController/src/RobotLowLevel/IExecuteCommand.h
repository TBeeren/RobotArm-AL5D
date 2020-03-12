/**
 * @file IExecuteCommand.h
 * @author Tim Beeren (T.Beeren1@student.han.nl)
 * @brief The ExecuteCommand interfaces gives higher level drivers the ability to send instructions to the robotarm.
 * @version 0.1
 * @date 06-03-2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef IEXECUTECOMMAND_H
#define IEXECUTECOMMAND_H

#include <iostream>
#include <vector>

// Enum Declaration
enum eCommand 
{
    MOVE_COMMAND = 0,
    CALIBRATE_COMMAND = 1,
    STOP_COMMAND = 2,
    UNKNOWN_COMMAND = 3
};

class IExecuteCommand
{
public:
    IExecuteCommand() = default;
    virtual ~IExecuteCommand() = default;

    /**
     * @brief This function will be called to stop communication to the robotarm
     * 
     */
    virtual void Stop() = 0;
    /**
     * @brief The write function is called to write a string message to the robotarm
     * 
     * @param rMessage the string which will be sent to the robotarm
     */
    virtual void Write(const std::string& rMessage) = 0;
    /**
     * @brief the AppendInstruction function adds an instruction to the list of instructions to send to the robotarm
     * 
     * @param eCommand the type of command to send to the servo
     * @param servo the servo to control
     * @param position the position to which the servo has to travel in PWM
     * @param speed the maximum speed of the movement
     * @param duration the duration of the movement
     */
    virtual void AppendInstruction(eCommand eCommand, int8_t servo , int64_t position, int64_t speed, int64_t duration) = 0;
    /**
     * @brief The execute function writes the stored instructions as a message to the robotarm 
     * 
     */
    virtual void Execute() = 0;
    /**
     * @brief The clearlists function clears all the previously appended instructions
     * 
     */
    virtual void ClearLists() = 0;

private:

};

#endif /*IEXECUTECOMMAND_H*/