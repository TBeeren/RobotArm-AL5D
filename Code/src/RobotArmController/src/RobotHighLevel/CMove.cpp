#include "CMove.h"

#include "../CServoInstruction.h"
#include "../RobotLowLevel/CCommandAL5D.h"

CMove::CMove()
: m_spExecuteCommand(std::make_shared<CCommandAL5D>())
{
}

CMove::~CMove()
{
}

void CMove::Execute(eCommand eCommand, std::vector<std::shared_ptr<CServoInstruction>> rServoIntructions)
{
    switch (eCommand)
    {
        case eCommand::MOVE:
        {
            Move(rServoIntructions);
            break;
        }
        case eCommand::STOP:
        {
            //Send Stop Command
            break;
        }
        case eCommand::UNKNOWN_COMMAND:
        {
            throw "Command is unknown! Please check your message on syntax";
            break;
        }
        default:
        {
            break;
        }
    }
}

void CMove::Move(std::vector<std::shared_ptr<CServoInstruction>> rServoIntructions)
{
    //m_spExecuteCommand
}
