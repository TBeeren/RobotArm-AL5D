#ifndef CMOVE_H
#define CMOVE_H

#include <vector>
#include <memory>
#include "../RobotLowLevel/IExecuteCommand.h"
<<<<<<< HEAD
=======
#include "CConfiguration.h"
>>>>>>> d0600a4d803aa1bafc6c67124fa9c1a123afe533

// Forward Declaration
class CServoInstruction;
class CCommandAL5D;

class CMove
{
public:
<<<<<<< HEAD
    CMove(std::shared_ptr<CConfiguration> spConfiguration);
    ~CMove();

    void Execute(eCommand eCommand, std::vector<std::shared_ptr<CServoInstruction>> rServoIntructions);
    void Move(std::vector<std::shared_ptr<CServoInstruction>> rServoIntructions);
    void Stop();

private:
    std::shared_ptr<IExecuteCommand> m_spExecuteCommand;
    std::shared_ptr<CConfiguration> m_spConfiguration;
=======
    CMove();
    ~CMove();

    void Execute(eCommand eCommand, std::vector<std::shared_ptr<CServoInstruction>> rServoInstructions);
    bool IsInstructionValid(std::shared_ptr<CServoInstruction> rServoInstruction);

private:
    uint16_t DegreesToPWM(eServos servo, int16_t degrees);

    std::shared_ptr<IExecuteCommand> m_spExecuteCommand;
>>>>>>> d0600a4d803aa1bafc6c67124fa9c1a123afe533

};

#endif /*CMOVE_H*/