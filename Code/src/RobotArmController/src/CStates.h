#ifndef CSTATES_H
#define CSTATES_H

class CIdleState
{
public:
    CIdleState();
    ~CIdleState();

    void Entry();
    void Do();
    void Exit();
private:
};

class CCalibrateState
{
public:
    CCalibrateState();
    ~CCalibrateState();

    void Entry();
    void Do();
    void Exit();
private:
};
class CMoveState
{
public:
    CMoveState();
    ~CMoveState();

    void Entry();
    void Do();
    void Exit();
private:
};

class CStopState
{
public:
    CStopState();
    ~CStopState();

    void Entry();
    void Do();
    void Exit();
private:
};

#endif /*CSTATES_H*/