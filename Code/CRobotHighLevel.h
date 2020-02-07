#ifndef CROBOTHIGHLEVEL_H
#define CROBOTHIGHLEVEL_H

#include <stdio.h>
#include <string>
#include <memory>

class CRobotLowLevel;

enum eGonioType
{
    DEGREE,
    RADIAN
};

class CRobotHighLevel
{
public:
    CRobotHighLevel(/* args */);
    ~CRobotHighLevel();
    void Move(uint16_t gonioValue, eGonioType gonioType, uint64_t time);

private:
    void TriggerEmergencyStop();
    uint16_t ToPwm(uint16_t gonioValue, eGonioType gonioType);
    
    std::shared_ptr<CRobotLowLevel> m_spCRobotLowLevel;
    uint16_t m_maxPWM;
};

#endif /* CROBOTHIGHLEVEL */
