/**
 * @file CStates.h
 * @author Evren Kilic (ET.Kilic@student.han.nl)
 * @brief In this file are the various states which the RobotArmController's state machine can be in
 * @version 0.1
 * @date 11-03-2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef CSTATES_H
#define CSTATES_H

#include "CEvent.h"
#include "IRobotStates.h"
#include <memory>

class CConfiguration;

class CIdleState: public IRobotStates
{
public:
/**
 * @brief The state a new RobotArmController is in and its in while it is idle
 * 
 * @param rEvent The event that triggered this state
 * @param spConfiguration the Configuration with the calibrated values for the robotarm, this will be passed when the handleEvent function is called onto other states.
 */
    CIdleState(CEvent& rEvent, std::shared_ptr<CConfiguration> spConfiguration);
    ~CIdleState();

    void Entry();
    void Do();
    void Exit();
    void HandleEvent(CEvent& rEvent, CRobotContext& rContext);
private:
    CEvent m_Event;
    std::shared_ptr<CConfiguration> m_spConfiguration;
};

class CCalibrateState: public IRobotStates
{
public:
/**
 * @brief The state a RobotArmController is in while it is calibrating movement in this state will be used in the configuration class as a calibration
 * 
 * @param rEvent The event that triggered this state
 * @param spConfiguration the Configuration with the calibrated values for the robotarm, this will be passed when the handleEvent function is called onto other states.
 */
    CCalibrateState(CEvent& rEvent, std::shared_ptr<CConfiguration> spConfiguration);
    ~CCalibrateState();

    void Entry();
    void Do();
    void Exit();
    void HandleEvent(CEvent& rEvent, CRobotContext& rContext);
private:
    CEvent m_Event;
    std::shared_ptr<CConfiguration> m_spConfiguration;
};
class CMoveState: public IRobotStates
{
public:
/**
 * @brief The state a RobotArmController is in when it recieves and executes a move command
 * 
 * @param rEvent The event that triggered this state
 * @param spConfiguration the Configuration with the calibrated values for the robotarm, this will be passed when the handleEvent function is called onto other states.
 */
    CMoveState(CEvent& rEvent, std::shared_ptr<CConfiguration> spConfiguration);
    ~CMoveState();

    void Entry();
    void Do();
    void Exit();
    void HandleEvent(CEvent& rEvent, CRobotContext& rContext);
private:
    CEvent m_Event;
    std::shared_ptr<CConfiguration> m_spConfiguration;
};

class CStopState: public IRobotStates
{
public:
/**
 * @brief The state a RobotArmController is in when the emergency stop is used. The application will need to be restarted after the controller is in this state.
 * 
 * @param rEvent The event that triggered this state
 * @param spConfiguration the Configuration with the calibrated values for the robotarm, this will be passed when the handleEvent function is called onto other states.
 */
    CStopState(CEvent& rEvent, std::shared_ptr<CConfiguration> spConfiguration);
    ~CStopState();

    void Entry();
    void Do();
    void Exit();
    void HandleEvent(CEvent& rEvent, CRobotContext& rContext);

private:
    CEvent m_Event;
    std::shared_ptr<CConfiguration> m_spConfiguration;
};

#endif /*CSTATES_H*/