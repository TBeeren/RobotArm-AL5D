#ifndef CEVENT_H
#define CEVENT_H

#include <vector>

class CServoInstruction;

enum eEventType
{
    IDLE,
    MOVE,
    CALIBRATE,
    EMERGENCY_STOP
};

class CEvent
{
public:
    explicit CEvent(eEventType eventType);
    explicit CEvent(eEventType eventType, bool preemptive, std::vector<CServoInstruction> servoInstructions);
    explicit CEvent(const CEvent& event);
    ~CEvent();

    bool IsPreemptive();
    eEventType GetEventType();
    std::vector<CServoInstruction> GetServoInstructions();
private:
    eEventType m_eventType;
    bool m_preemptive;
    std::vector<CServoInstruction> m_servoInstructions;
};

#endif /*CEVENT_H*/
