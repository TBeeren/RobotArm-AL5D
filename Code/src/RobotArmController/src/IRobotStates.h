#ifndef IROBOTSTATES_H
#define IROBOTSTATES_H

class CEvent;
class CRobotContext;

class IRobotStates
{
public:
    IRobotStates() = default;
    virtual ~IRobotStates() = default;
    virtual void Entry() = 0;
    virtual void Do() = 0;
    virtual void Exit() = 0;
    virtual void HandleEvent(CEvent& rEvent, CRobotContext& rContext) = 0;
protected:
private:
};

#endif /*IROBOTSTATES_H*/