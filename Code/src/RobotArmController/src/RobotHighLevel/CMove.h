#ifndef CMOVE_H
#define CMOVE_H

#include <vector>
#include <memory>

// Forward Declaration
class CServoInstruction;
class CCommandAL5D;
class IExecuteCommand;

// Enum Declaration
enum eCommand 
{
    MOVE = 0,
    STOP= 1,
    UNKNOWN_COMMAND = 2
};

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