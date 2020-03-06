#ifndef IEXECUTECOMMAND_H
#define IEXECUTECOMMAND_H

#include <iostream>

namespace
{
    constexpr const uint8_t INSTRUCTION_ARRAY_SIZE = 5;
}

class IExecuteCommand
{
public:
    IExecuteCommand();
    ~IExecuteCommand();

    virtual void Move() = 0;
private:

    uint64_t m_positionArray[INSTRUCTION_ARRAY_SIZE];
    uint64_t m_speedArray[INSTRUCTION_ARRAY_SIZE];
    uint64_t m_durationArray[INSTRUCTION_ARRAY_SIZE];

};

#endif /*IEXECUTECOMMAND_H*/