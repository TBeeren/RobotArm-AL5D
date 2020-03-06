#ifndef CEVENT_H
#define CEVENT_H

#include <vector>
#include <memory>

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
    explicit CEvent(eEventType eventType, bool preemptive, std::vector<std::shared_ptr<CServoInstruction>> servoInstructions);
    explicit CEvent(const CEvent& event);
    ~CEvent();

    bool IsPreemptive();
    eEventType GetEventType();
    std::vector<std::shared_ptr<CServoInstruction>> GetServoInstructions();
private:
    eEventType m_eventType;
    bool m_preemptive;
    std::vector<std::shared_ptr<CServoInstruction>> m_spServoInstructions;
};

#endif /*CEVENT_H*/
