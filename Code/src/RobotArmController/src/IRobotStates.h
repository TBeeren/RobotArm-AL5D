#ifndef IROBOTSTATES_H
#define IROBOTSTATES_H

class IRobotStates
{
public:
    IRobotStates() = default;
    virtual ~IRobotStates() = default;
protected:
    virtual void Entry() = 0;
    virtual void Do() = 0;
    virtual void Exit() = 0;
private:
};

#endif /*IROBOTSTATES_H*/