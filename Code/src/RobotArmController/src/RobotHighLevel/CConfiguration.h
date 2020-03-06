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