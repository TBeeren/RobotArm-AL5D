<<<<<<< HEAD
#ifndef CCONFIGURATION_H
#define CCONFIGURATION_H

#include <map>
=======
/**
 * @file CConfiguration.h
 * @author Tim Beeren (T.Beeren1@student.han.nl)
 * @brief 
 * @version 0.1
 * @date 06-03-2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */


#ifndef CCONFIGURATION_H
#define CCONFIGURATION_H

#include <memory>
#include <map>
#include <vector>

// Forward Declaration
class CConfiguration;
>>>>>>> d0600a4d803aa1bafc6c67124fa9c1a123afe533

// Enum Declaration
enum eServos 
{
    BASE = 0,
    SHOULDER = 1,
    ELBOW = 2,
    WRIST = 3,
    GRIPPER = 4,
    WRIST_ROTATE = 5,
<<<<<<< HEAD
    UNKNOWN = 6
};

enum eProgrammedPosition
{
    PARK,
    READY,
    STRAIGHT
};


class CConfiguration
{
public:
    CConfiguration(/* args */);
    ~CConfiguration();

    eProgrammedPosition StringToProgrammedPosition(std::string programmedPositionString);
private:
    std::map<eProgrammedPosition, std::string> programmedPositions;
    /* data */
};

#endif /*CONFIGURATION_H*/
=======
    UNKNOWN_SERVO = 6
};

class CConfiguration
{
public:
    CConfiguration(std::shared_ptr<CConfiguration> spConfiguration);
    ~CConfiguration();

    // Default configuration
    std::vector<uint16_t> m_baseConfig;
    std::vector<uint16_t> m_shoulderConfig;
    std::vector<uint16_t> m_elbowConfig;
    std::vector<uint16_t> m_wristConfig;
    std::vector<uint16_t> m_gripperConfig;
    std::vector<uint16_t> m_wristRotateConfig;

    // Configuration
    std::map<eServos, std::vector<uint16_t>> configuredServos;

    // Methods
    void ExecuteMovement();
    void Write(eServos eServo, uint16_t minValue, uint16_t maxValue);
    
private:
    std::shared_ptr<CConfiguration> m_spConfiguration;
};

#endif /*CCONFIGURATION_H*/
>>>>>>> d0600a4d803aa1bafc6c67124fa9c1a123afe533
