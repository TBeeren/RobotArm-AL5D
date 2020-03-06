#include "CCalibration.h"

#include "../CServoInstruction.h"
#include "CMove.h"

CCalibration::CCalibration(std::shared_ptr<CConfiguration> spConfiguration, std::shared_ptr<CMove> spMove)
: m_spConfiguration(spConfiguration)
, m_spMove(spMove)
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