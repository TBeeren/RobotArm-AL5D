#ifndef CROBOTLOWLEVEL_H
#define CROBOTLOWLEVEL_H

#include <string>

class CRobotLowLevel
{
public:
    CRobotLowLevel(/* args */);
    ~CRobotLowLevel();
    bool Move(uint16_t pwm, uint64_t time);
    uint16_t GetMaxPWM();

private:
    std::string ToProtocolMessage(uint16_t pwm, uint64_t time);
    bool IsHardwareCompatible(uint16_t pwm);
    bool WriteToSerialPort(const std::string& message);

    uint16_t m_maxPWM = 180;
    std::string m_serialPort = "/dev/ttyACM0";
};

#endif /* CROBOTLOWLEVEL */
