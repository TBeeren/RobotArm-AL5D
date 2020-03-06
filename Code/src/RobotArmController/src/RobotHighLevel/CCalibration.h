#ifndef CCALIBRATION_H
#define CCALIBRATION_H

#include <memory>
#include <vector>

#include "CConfiguration.h"

// Forward Declaration
class CServoInstruction;
class CMove;

class CCalibration
{
public:
    explicit CCalibration(std::shared_ptr<CConfiguration> spConfiguration);
    virtual ~CCalibration();

    void ExecuteMovement(std::vector<std::shared_ptr<CServoInstruction>> rServoIntructions);

private:
    bool WriteConfig(eServos eServo, uint16_t minValue, uint16_t maxValue);
    std::shared_ptr<CConfiguration> m_spConfiguration;
    std::shared_ptr<CMove> m_spMove;
    
};

#endif /*CCALIBRATION_H*/
