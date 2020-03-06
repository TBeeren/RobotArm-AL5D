#include "CConfiguration.h"

//Anonymous namespace
namespace 
{
    constexpr uint16_t minBaseValue = 0; 
    constexpr uint16_t maxBaseValue = 1023;
    constexpr uint16_t minShoulderValue = 0; 
    constexpr uint16_t maxShoulderValue = 1023;
    constexpr uint16_t minElbowValue = 0; 
    constexpr uint16_t maxElbowValue = 1023;
    constexpr uint16_t minWristValue = 0; 
    constexpr uint16_t maxWristValue = 1023;
    constexpr uint16_t minGripperValue = 0; 
    constexpr uint16_t maxGripperValue = 1023;
    constexpr uint16_t minWristRotateValue = 0;
    constexpr uint16_t maxWristRotateValue = 1023;

    constexpr const uint8_t minValueVectorIndex = 0;
    constexpr const uint8_t maxValueVectorIndex = 1;
}

CConfiguration::CConfiguration(std::shared_ptr<CConfiguration> spConfiguration)
: m_spConfiguration(spConfiguration)
, m_baseConfig(minBaseValue, maxBaseValue)
, m_shoulderConfig(minShoulderValue, maxShoulderValue)
, m_elbowConfig(minElbowValue, maxElbowValue)
, m_wristConfig(minWristValue, maxWristValue)
, m_gripperConfig(minGripperValue, maxGripperValue)
, m_wristRotateConfig(minGripperValue, maxGripperValue)
{
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


void CConfiguration::Write(eServos eServo, uint16_t minValue, uint16_t maxValue)
{
    switch (eServo)
    {
        case eServos::BASE:
        {
            m_baseConfig[minValueVectorIndex] = minValue;
            m_baseConfig[maxValueVectorIndex] = maxValue;
            break;
        }
        case eServos::SHOULDER:
        {
            m_shoulderConfig[minValueVectorIndex] = minValue;
            m_shoulderConfig[maxValueVectorIndex] = maxValue;
            break;
        }
        case eServos::ELBOW:
        {
            m_elbowConfig[minValueVectorIndex] = minValue;
            m_elbowConfig[maxValueVectorIndex] = maxValue;
            break;
        }
        case eServos::WRIST:
        {
            m_wristConfig[minValueVectorIndex] = minValue;
            m_wristConfig[maxValueVectorIndex] = maxValue;
            break;
        }
        case eServos::GRIPPER:
        {
            m_gripperConfig[minValueVectorIndex] = minValue;
            m_gripperConfig[maxValueVectorIndex] = maxValue;
            break;
        }
        case eServos::WRIST_ROTATE:
        {
            m_wristConfig[minValueVectorIndex] = minValue;
            m_wristConfig[maxValueVectorIndex] = maxValue;
            break;
        }
        case eServos::UNKNOWN_SERVO:
        {
            throw "Servo type is unknown! Please check your message on syntax";
            break;
        }
        default:
        {
            break;
        }
    }
}
     