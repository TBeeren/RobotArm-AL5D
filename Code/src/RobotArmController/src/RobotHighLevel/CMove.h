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

    void Execute(eCommand eCommand, std::vector<std::shared_ptr<CServoInstruction>> rServoInstructions);
    void ExecuteCalibrate(eCommand eCommand, std::vector<std::shared_ptr<CServoInstruction>> rServoInstructions);
    bool IsInstructionValid(std::shared_ptr<CServoInstruction> rServoInstruction);
    uint16_t DegreesToPWM(eServos servo, int16_t degrees);
    uint16_t CalibrationDegreesToPwm(int16_t degrees);
    std::vector<int> GetMinAndMaxValue(std::shared_ptr<CServoInstruction> instruction);

private:
    std::shared_ptr<CConfiguration> m_spConfiguration;
    std::shared_ptr<IExecuteCommand> m_spExecuteCommand;
};

#endif /*CMOVE_H*/