#ifndef CCONFIGURATION_H
#define CCONFIGURATION_H

#include <map>

// Enum Declaration
enum eServos 
{
    BASE = 0,
    SHOULDER = 1,
    ELBOW = 2,
    WRIST = 3,
    GRIPPER = 4,
    WRIST_ROTATE = 5,
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