
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

    virtual void Move() = 0;
    virtual void Stop() = 0;
    virtual void Write(const std::string& rMessage) = 0;
    virtual void AppendInstruction(eCommand eCommand, uint64_t position, uint64_t speed, uint64_t duration) = 0;

private:

};

#endif /*IEXECUTECOMMAND_H*/