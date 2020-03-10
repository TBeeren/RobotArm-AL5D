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
    CMove();
    ~CMove();

    void Execute(eCommand eCommand, std::vector<std::shared_ptr<CServoInstruction>> rServoInstructions);
    bool IsInstructionValid(std::shared_ptr<CServoInstruction> rServoInstruction);

private:
    uint16_t DegreesToPWM(eServos servo, int16_t degrees);

    std::shared_ptr<IExecuteCommand> m_spExecuteCommand;

};

#endif /*CMOVE_H*/