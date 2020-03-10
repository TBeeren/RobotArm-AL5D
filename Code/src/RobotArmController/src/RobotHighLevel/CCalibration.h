<<<<<<< HEAD
=======
/**
 * @file CCalibration.h
 * @author Tim Beeren (T.Beeren1@student.han.nl)
 * @brief 
 * @version 0.1
 * @date 06-03-2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */

>>>>>>> d0600a4d803aa1bafc6c67124fa9c1a123afe533
#ifndef CCALIBRATION_H
#define CCALIBRATION_H

#include <memory>
#include <vector>

#include "CConfiguration.h"
<<<<<<< HEAD
=======
#include "../RobotLowLevel/IExecuteCommand.h"
>>>>>>> d0600a4d803aa1bafc6c67124fa9c1a123afe533

// Forward Declaration
class CServoInstruction;
class CMove;

class CCalibration
{
public:
    explicit CCalibration(std::shared_ptr<CConfiguration> spConfiguration);
    virtual ~CCalibration();

<<<<<<< HEAD
    void ExecuteMovement(std::vector<std::shared_ptr<CServoInstruction>> rServoIntructions);
=======
    void Execute(eCommand eCommand, std::vector<std::shared_ptr<CServoInstruction>> rServoIntructions);
>>>>>>> d0600a4d803aa1bafc6c67124fa9c1a123afe533

private:
    bool WriteConfig(eServos eServo, uint16_t minValue, uint16_t maxValue);
    std::shared_ptr<CConfiguration> m_spConfiguration;
    std::shared_ptr<CMove> m_spMove;
    
};

<<<<<<< HEAD
#endif /*CCALIBRATION_H*/
=======
#endif /*CCALIBRATION_H*/
>>>>>>> d0600a4d803aa1bafc6c67124fa9c1a123afe533
