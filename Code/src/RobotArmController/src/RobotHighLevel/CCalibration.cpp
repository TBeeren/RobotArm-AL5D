/**
 * @file CCalibration.cpp
 * @author Tim Beeren (T.Beeren1@student.han.nl)
 * @brief 
 * @version 0.1
 * @date 06-03-2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "CCalibration.h"

#include "../CServoInstruction.h"
#include "CMove.h"

CCalibration::CCalibration(std::shared_ptr<CConfiguration> spConfiguration)
: m_spConfiguration(spConfiguration)
, m_spMove(std::make_shared<CMove>())
{
}

CCalibration::~CCalibration()
{
}

void CCalibration::ExecuteMovement(std::vector<std::shared_ptr<CServoInstruction>> rServoIntructions)
{   
    //m_spMove->Execute();
}
    
bool CCalibration::WriteConfig(eServos eServo, uint16_t minValue, uint16_t maxValue)
{
    m_spConfiguration->Write(eServo, minValue, maxValue);
}