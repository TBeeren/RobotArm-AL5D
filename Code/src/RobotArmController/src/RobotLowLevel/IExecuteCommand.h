<<<<<<< HEAD
=======
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
>>>>>>> d0600a4d803aa1bafc6c67124fa9c1a123afe533

#ifndef IEXECUTECOMMAND_H
#define IEXECUTECOMMAND_H

#include <iostream>

// Enum Declaration
enum eCommand 
{
    MOVE_COMMAND = 0,
    STOP_COMMAND = 1,
    UNKNOWN_COMMAND = 2
};

class IExecuteCommand
{
public:
    IExecuteCommand();
    ~IExecuteCommand();

<<<<<<< HEAD
    virtual void Move() = 0;
    virtual void Stop() = 0;
    virtual void Write(const std::string& rMessage) = 0;
    virtual void AppendInstruction(eCommand eCommand, uint64_t position, uint64_t speed, uint64_t duration) = 0;
=======
    virtual void Stop() = 0;
    virtual void Write(const std::string& rMessage) = 0;
    virtual void AppendInstruction(eCommand eCommand, uint64_t position, uint64_t speed, uint64_t duration) = 0;
    virtual void ClearLists() = 0;
>>>>>>> d0600a4d803aa1bafc6c67124fa9c1a123afe533

private:

};

#endif /*IEXECUTECOMMAND_H*/