/**
 * @file CConfiguration.cpp
 * @author Tim Beeren (T.Beeren1@student.han.nl)
 * @brief 
 * @version 0.1
 * @date 06-03-2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "CConfiguration.h"
#include <iostream>

//Anonymous namespace
namespace 
{
    constexpr uint16_t minPWMValue = 1250; 
    constexpr uint16_t maxPWMValue = 1750;

    constexpr const uint8_t minValueVectorIndex = 0;
    constexpr const uint8_t maxValueVectorIndex = 1;
}

CConfiguration::CConfiguration()
: m_baseConfig(minPWMValue, maxPWMValue)
, m_shoulderConfig(minPWMValue, maxPWMValue)
, m_elbowConfig(minPWMValue, maxPWMValue)
, m_wristConfig(minPWMValue, maxPWMValue)
, m_gripperConfig(minPWMValue, maxPWMValue)
, m_wristRotateConfig(minPWMValue, maxPWMValue)
{
    programmedPositions =
    {
        {eProgrammedPosition::PARK, "park"},
        {eProgrammedPosition::READY, "ready"},
        {eProgrammedPosition::STRAIGHT, "straight"}
    };

    // Initialising the map with servo configuration.
    configuredServos =
    {
        {eServos::BASE, m_baseConfig},
        {eServos::SHOULDER, m_shoulderConfig},
        {eServos::ELBOW, m_elbowConfig},
        {eServos::WRIST , m_wristConfig},
        {eServos::GRIPPER , m_gripperConfig},
        {eServos::WRIST_ROTATE , m_wristRotateConfig}
    };
}

CConfiguration::~CConfiguration()
{
}


void CConfiguration::Write(eServos eServo, uint16_t value)
{
    switch (eServo)
    {
        case eServos::BASE:
        {
            if(value < m_baseConfig[minValueVectorIndex])
            {
                m_baseConfig[minValueVectorIndex] = value;
            }
            else if(value > m_baseConfig[maxValueVectorIndex])
            {
                m_baseConfig[maxValueVectorIndex] = value;
            }
            break;
        }
        case eServos::SHOULDER:
        {
            if(value < m_shoulderConfig[minValueVectorIndex])
            {
                m_shoulderConfig[minValueVectorIndex] = value;
            }
            else if(value > m_shoulderConfig[maxValueVectorIndex])
            {
                m_shoulderConfig[maxValueVectorIndex] = value;
            }
            break;
        }
        case eServos::ELBOW:
        {
            if(value < m_elbowConfig[minValueVectorIndex])
            {
                m_elbowConfig[minValueVectorIndex] = value;
            }
            else if(value > m_elbowConfig[maxValueVectorIndex])
            {
                m_elbowConfig[maxValueVectorIndex] = value;
            }
            break;
        }
        case eServos::WRIST:
        {
            if(value < m_wristConfig[minValueVectorIndex])
            {
                m_wristConfig[minValueVectorIndex] = value;
            }
            else if(value > m_wristConfig[maxValueVectorIndex])
            {
                m_wristConfig[maxValueVectorIndex] = value;
            }
            break;
        }
        case eServos::GRIPPER:
        {
            if(value < m_gripperConfig[minValueVectorIndex])
            {
                m_gripperConfig[minValueVectorIndex] = value;
            }
            else if(value > m_gripperConfig[maxValueVectorIndex])
            {
                m_gripperConfig[maxValueVectorIndex] = value;
            }
            break;
        }
        case eServos::WRIST_ROTATE:
        {
            if(value < m_wristRotateConfig[minValueVectorIndex])
            {
                m_wristRotateConfig[minValueVectorIndex] = value;
            }
            else if(value > m_wristRotateConfig[maxValueVectorIndex])
            {
                m_wristRotateConfig[maxValueVectorIndex] = value;
            }
            break;
        }
        case eServos::UNKNOWN_SERVO:
        {
            std::cout << " [Warning]-> Servo type is unknown! Please check your message on syntax" << std::endl;
            break;
        }
        default:
        {
            break;
        }
    }
}

uint16_t CConfiguration::GetMinPWM(eServos eServo)
{
    uint16_t returnValue;
    switch (eServo)
    {
        case eServos::BASE:
        {
            returnValue = m_baseConfig[minValueVectorIndex];
            break;
        }
        case eServos::SHOULDER:
        {
            returnValue = m_shoulderConfig[minValueVectorIndex];
            break;
        }
        case eServos::ELBOW:
        {
            returnValue = m_elbowConfig[minValueVectorIndex];
            break;
        }
        case eServos::WRIST:
        {
            returnValue = m_wristConfig[minValueVectorIndex];
            break;
        }
        case eServos::GRIPPER:
        {
            returnValue = m_gripperConfig[minValueVectorIndex];
            break;
        }
        case eServos::WRIST_ROTATE:
        {
            returnValue = m_wristRotateConfig[minValueVectorIndex];
            break;
        }
        case eServos::UNKNOWN_SERVO:
        {
            std::cout << "Servo type is unknown! Please check your message on syntax" << std::endl;
            break;
        }
        default:
        {
            std::cout << "Servo type is unknown! Please check your message on syntax" << std::endl;
            break;
        }
    }
    return returnValue;
}

uint16_t CConfiguration::GetMaxPWM(eServos eServo)
{
    uint16_t returnValue;
    switch (eServo)
    {
        case eServos::BASE:
        {
            returnValue = m_baseConfig[maxValueVectorIndex];
            break;
        }
        case eServos::SHOULDER:
        {
            returnValue = m_shoulderConfig[maxValueVectorIndex];
            break;
        }
        case eServos::ELBOW:
        {
            returnValue = m_elbowConfig[maxValueVectorIndex];
            break;
        }
        case eServos::WRIST:
        {
            returnValue = m_wristConfig[maxValueVectorIndex];
            break;
        }
        case eServos::GRIPPER:
        {
            returnValue = m_gripperConfig[maxValueVectorIndex];
            break;
        }
        case eServos::WRIST_ROTATE:
        {
            returnValue = m_wristRotateConfig[maxValueVectorIndex];
            break;
        }
        case eServos::UNKNOWN_SERVO:
        {
            std::cout << "Servo type is unknown! Please check your message on syntax" << std::endl;
            break;
        }
        default:
        {
            std::cout << "Servo type is unknown! Please check your message on syntax" << std::endl;
            break;
        }
    }
    return returnValue;
}

eProgrammedPosition CConfiguration::StringToProgrammedPosition(std::string programmedPositionString)
{
    for(auto programmedPosition : programmedPositions)
    {
        if(programmedPosition.second == programmedPositionString)
        {
            return programmedPosition.first;
        }
    }
}
     
