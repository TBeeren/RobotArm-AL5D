#include "CRobotHighLevel.h"
#include "CRobotLowLevel.h"
#include <math.h>

namespace 
{
    constexpr const uint16_t DEGREES_IN_CIRCLE = 360;
    constexpr const float RADIANS_IN_CIRCLE = 2 * M_PI;
}


CRobotHighLevel::CRobotHighLevel()
: m_spCRobotLowLevel(std::make_shared<CRobotLowLevel>())
, m_maxPWM(m_spCRobotLowLevel->GetMaxPWM())
{
}

CRobotHighLevel::~CRobotHighLevel()
{
}

void CRobotHighLevel::Move(uint16_t gonioValue, eGonioType gonioType, uint64_t time)
{
    m_spCRobotLowLevel->Move(ToPwm(gonioValue, gonioType), time);
}

void CRobotHighLevel::TriggerEmergencyStop()
{
    m_spCRobotLowLevel->Move(0,0);
}

uint16_t CRobotHighLevel::ToPwm(uint16_t gonioValue, eGonioType gonioType)
{
    uint16_t returnValue = 0;

    switch(gonioType)
    {
        case eGonioType::DEGREE:
        {
            returnValue = ((m_maxPWM / DEGREES_IN_CIRCLE) * gonioValue);
            break;
        }
        case eGonioType::RADIAN:
        {
            returnValue = ((static_cast<float>(m_maxPWM)) / RADIANS_IN_CIRCLE * gonioValue);
            break;
        }
    }
    return returnValue;
}