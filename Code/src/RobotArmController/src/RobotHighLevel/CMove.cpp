/**
 * @file CMove.cpp
 * @author Tim Beeren (T.Beeren1@student.han.nl)
 * @brief 
 * @version 0.1
 * @date 06-03-2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "CMove.h"

#include "../CServoInstruction.h"
#include "../RobotLowLevel/CCommandAL5D.h"
#include <assert.h>

CMove::CMove()
: m_spExecuteCommand(std::make_shared<CCommandAL5D>())
{
}

CMove::~CMove()
{
}

void CMove::Execute(eCommand eCommand, std::vector<std::shared_ptr<CServoInstruction>> rServoInstructions)
{
    assert(rServoInstructions.size() != 0);

    for(const auto& rInstruction : rServoInstructions)
    {
        m_spExecuteCommand->AppendInstruction(
            eCommand, 
            rInstruction->GetTargetServo(), 
            rInstruction->GetSpeed(), 
            rInstruction->GetDuration()
        );
    }
}