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
    CMove(std::shared_ptr<CConfiguration> spConfiguration);
    ~CMove();

    void Execute(eCommand eCommand, std::vector<std::shared_ptr<CServoInstruction>> rServoIntructions);
    void Move(std::vector<std::shared_ptr<CServoInstruction>> rServoIntructions);
    void Stop();

private:
    std::shared_ptr<IExecuteCommand> m_spExecuteCommand;
    std::shared_ptr<CConfiguration> m_spConfiguration;

};

#endif /*CMOVE_H*/