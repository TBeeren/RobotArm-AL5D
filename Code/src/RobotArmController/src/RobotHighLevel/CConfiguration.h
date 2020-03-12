/**
 * @file CConfiguration.h
 * @author Tim Beeren (T.Beeren1@student.han.nl)
 * @brief The configuration class holds the calibrated values for every servo
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

// Enum Declaration
enum eServos 
{
    BASE = 0,
    SHOULDER = 1,
    ELBOW = 2,
    WRIST = 3,
    GRIPPER = 4,
    WRIST_ROTATE = 5,
    UNKNOWN_SERVO = 6
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
    CConfiguration();
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
    /**
     * @brief Writes calibration values for a gives servo
     * 
     * @param eServo the Servo onto which the calibration implies
     * @param value the value of the calibration
     */
    void Write(eServos eServo, uint16_t value);
    /**
     * @brief Gets the minimal pwm value for a given servo
     * 
     * @param eServo the Servo for which to get the minimal pwm value
     * @return uint16_t the minimal pwm value
     */
    uint16_t GetMinPWM(eServos eServo);
    /**
     * @brief Gets the maximum pwm value for a given servo
     * 
     * @param eServo the Servo for which to get the maximum pwm value
     * @return uint16_t the maximum pwm value
     */
    uint16_t GetMaxPWM(eServos eServo);
    /**
     * @brief translates a recieved string into a programmed position if one is known
     * 
     * @param programmedPositionString the string to be translated into a programmed position
     * @return eProgrammedPosition the programmed position that was found matching the string, if any was found
     */
    eProgrammedPosition StringToProgrammedPosition(std::string programmedPositionString);

private:
    std::map<eProgrammedPosition, std::string> programmedPositions;
};

#endif /*CCONFIGURATION_H*/
