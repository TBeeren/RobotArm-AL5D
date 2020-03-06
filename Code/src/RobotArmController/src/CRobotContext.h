/**
 * @file CRobotContext.h
 * @author Evren Kilic (ET.Kilic@student.han.nl)
 * @brief 
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

    void Init();
    void Run();
    void SetState(std::shared_ptr<IRobotStates> sp_state);
    void MoveCallback(const RobotArmController::Move::ConstPtr& moveMsg);
    void CalibrateCallback(const RobotArmController::Move::ConstPtr& calibrateMsg);
    void EmergencyStopCallback(const RobotArmController::EmergencyStop::ConstPtr& stopMsg);
    void ProgrammedPositionCallback(const RobotArmController::ProgrammedPosition::ConstPtr& programmedPositionMsg);
private:
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