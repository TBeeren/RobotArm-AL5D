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
// : m_spConfiguration(spConfiguration)
// , m_spMove(std::make_shared<CMove>(spConfiguration))
{
    m_spConfiguration = spConfiguration;
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    m_spMove = std::make_shared<CMove>(spConfiguration);
}

CCalibration::~CCalibration()
{
}

void CCalibration::Execute(eCommand eCommand, std::vector<std::shared_ptr<CServoInstruction>> rServoIntructions)
{   
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    m_spMove->Execute(eCommand, rServoIntructions);
}
    
bool CCalibration::WriteConfig(eServos eServo, uint16_t minValue, uint16_t maxValue)
{
    m_spConfiguration->Write(eServo, minValue, maxValue);
    return true;
}