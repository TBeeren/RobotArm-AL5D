#ifndef CSTATES_H
#define CSTATES_H

#include "CEvent.h"
#include "IRobotStates.h"
#include <memory>

class CConfiguration;

class CIdleState: public IRobotStates
{
public:
    CIdleState(CEvent& rEvent, std::shared_ptr<CConfiguration> spConfiguration);
    ~CIdleState();

    void Entry();
    void Do();
    void Exit();
    void HandleEvent(CEvent& rEvent, CRobotContext& rContext);
private:
    CEvent& m_rEvent;
    std::shared_ptr<CConfiguration> m_spConfiguration;
};

class CCalibrateState: public IRobotStates
{
public:
    CCalibrateState(CEvent& rEvent, std::shared_ptr<CConfiguration> spConfiguration);
    ~CCalibrateState();

    void Entry();
    void Do();
    void Exit();
    void HandleEvent(CEvent& rEvent, CRobotContext& rContext);
private:
    CEvent& m_rEvent;
    std::shared_ptr<CConfiguration> m_spConfiguration;
};
class CMoveState: public IRobotStates
{
public:
    CMoveState(CEvent& rEvent, std::shared_ptr<CConfiguration> spConfiguration);
    ~CMoveState();

    void Entry();
    void Do();
    void Exit();
    void HandleEvent(CEvent& rEvent, CRobotContext& rContext);
private:
    CEvent& m_rEvent;
    std::shared_ptr<CConfiguration> m_spConfiguration;
};

class CStopState: public IRobotStates
{
public:
    CStopState(CEvent& rEvent, std::shared_ptr<CConfiguration> spConfiguration);
    ~CStopState();

    void Entry();
    void Do();
    void Exit();
    void HandleEvent(CEvent& rEvent, CRobotContext& rContext);
private:
    CEvent& m_rEvent;
    std::shared_ptr<CConfiguration> m_spConfiguration;
};

#endif /*CSTATES_H*/