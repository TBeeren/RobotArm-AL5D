/**
 * @file CRobotContext.h
 * @author Evren Kilic (ET.Kilic@student.han.nl)
 * @brief The CRobotContext is the context for the state machine which controlls the robot arm through its states of moving, idling, calibrating, and stopping.
 * It also holds a nodehandle with which it subscribes to ros topics on which instructions get published.
 * @version 0.1
 * @date 05-03-2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef CROBOTCONTEXT_H
#define CROBOTCONTEXT_H

#include <ros/ros.h>
#include <memory>
#include <queue>
#include <chrono>
#include "CEvent.h"
#include "RobotArmController/Move.h"
#include "RobotArmController/EmergencyStop.h"
#include "RobotArmController/ProgrammedPosition.h"

class IRobotStates;
class CConfiguration;

class CRobotContext
{
public:
    CRobotContext();
    ~CRobotContext();

    /**
     * @brief The Run function runs the state machine so it handles all the events in its queue, afterwards it will stay waiting for new events to be created by functions to
     * be called by ros
     * 
     */
    void Run();
    /**
     * @brief Sets the state of the Context, in doing so it goes through the exit action of the previous state, the entry action of the new state and into the do activity
     * of the new state. All in that order
     * 
     * @param sp_state a shared pointer to the new state for this context
     */
    void SetState(std::shared_ptr<IRobotStates> sp_state);

    /**
     * @brief The callback function which will be called when a message gets posted on the /RobotArmController/Move ros topic
     * 
     * @param moveMsg a message containing instructions for the individual servos of the robotarm
     */
    void MoveCallback(const RobotArmController::Move::ConstPtr& moveMsg);
    /**
     * @brief The callback function which will be called when a message gets posted on the /RobotArmController/Calibrate ros topic
     * 
     * @param calibrateMsg a message containing instructions for the individual servos of the robotarm. these values will be used to calibrate new boundries
     */
    void CalibrateCallback(const RobotArmController::Move::ConstPtr& calibrateMsg);
    /**
     * @brief The callback function which will be called when a message gets posted on the /RobotArmController/EmergencyStop ros topic
     * 
     * @param stopMsg an empty message
     */
    void EmergencyStopCallback(const RobotArmController::EmergencyStop::ConstPtr& stopMsg);
    /**
     * @brief The callback function which will be called when a message gets posted on the /RobotArmController/ProgrammedPosition ros topic
     * 
     * @param programmedPositionMsg a message containing a string to set pre defined positions
     */
    void ProgrammedPositionCallback(const RobotArmController::ProgrammedPosition::ConstPtr& programmedPositionMsg);
private:
    /**
     * @brief Gets a duration for the servoinstruction with the highest duration in an event
     * 
     * @param rEvent the event which holds the servoinstructions
     * @return std::chrono::duration<double> a time duration as specified in the std::chrono library which is the highest duration amongst all servo instructions
     */
    std::chrono::duration<double> GetHighestDuration(CEvent& rEvent);

    std::shared_ptr<IRobotStates> m_spCurrentState;
    std::shared_ptr<CConfiguration> m_spConfiguration;
    ros::NodeHandle m_nodeHandle;
    ros::Subscriber m_moveSubcriber;
    ros::Subscriber m_calibrateSubscriber;
    ros::Subscriber m_emergencyStopSubcriber;
    ros::Subscriber m_programmedPositionSubcriber;
    std::queue<CEvent> m_preemptiveEventQueue;
    std::queue<CEvent> m_queuedEventQueue;
};

#endif /*CROBOTCONTEXT_H*/