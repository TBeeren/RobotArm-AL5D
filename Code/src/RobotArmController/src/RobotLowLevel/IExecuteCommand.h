/**
 * @file IExecuteCommand.h
 * @author Tim Beeren (T.Beeren1@student.han.nl)
 * @brief 
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

    virtual void Stop() = 0;
    virtual void Write(const std::string& rMessage) = 0;
    virtual void AppendInstruction(eCommand eCommand, int8_t servo , int64_t position, int64_t speed, int64_t duration) = 0;
    virtual void Execute() = 0;
    virtual void ClearLists() = 0;

private:

};

#endif /*IEXECUTECOMMAND_H*/