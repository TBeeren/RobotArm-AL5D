/**
 * @file CEvent.h
 * @author Evren Kilic (ET.Kilic@student.han.nl)
 * @brief The CEvent class is used as an event for the State machine, it holds instructions for servos and an eventtype to be used in the state machine.
 * @version 0.1
 * @date 11-03-2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
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

    /**
     * @brief Returns wether this event is ment to be preemptive or not
     * 
     * @return true the event is preemptive
     * @return false the event isn't preemptive
     */
    bool IsPreemptive();
    eEventType GetEventType();
    std::vector<std::shared_ptr<CServoInstruction>> GetServoInstructions();
private:
    eEventType m_eventType;
    bool m_preemptive;
    std::vector<std::shared_ptr<CServoInstruction>> m_spServoInstructions;
};

#endif /*CEVENT_H*/
