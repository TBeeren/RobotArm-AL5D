#ifndef CCONFIGURATION_H
#define CCONFIGURATION_H

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

class CConfiguration
{
private:
    /* data */
public:
    CConfiguration(/* args */);
    ~CConfiguration();
};

#endif /*CONFIGURATION_H*/