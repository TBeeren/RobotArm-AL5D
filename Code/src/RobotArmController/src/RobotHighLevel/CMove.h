#ifndef CMOVE_H
#define CMOVE_H

#include <vector>
#include <memory>
#include "../RobotLowLevel/IExecuteCommand.h"

// Forward Declaration
class CServoInstruction;
class CCommandAL5D;

class CMove
{
public:
    CMove();
    ~CMove();

    void Execute(eCommand eCommand, std::vector<std::shared_ptr<CServoInstruction>> rServoIntructions);
    void Move(std::vector<std::shared_ptr<CServoInstruction>> rServoIntructions);

private:
    std::shared_ptr<IExecuteCommand> m_spExecuteCommand;

};

#endif /*CMOVE_H*/