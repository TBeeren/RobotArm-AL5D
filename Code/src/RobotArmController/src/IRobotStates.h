/**
 * @file IRobotStates.h
 * @author Evren Kilic (ET.Kilic@student.han.nl)
 * @brief The interface for the various classes in the state machine
 * @version 0.1
 * @date 11-03-2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef IROBOTSTATES_H
#define IROBOTSTATES_H

class CEvent;
class CRobotContext;

class IRobotStates
{
public:
    IRobotStates() = default;
    virtual ~IRobotStates() = default;
    /**
     * @brief the entry action which will be executed for the various states when the context enters that state.
     * 
     */
    virtual void Entry() = 0;

    /**
     * @brief The do activity which will be executed after the entry action for the various states when the context enters that state.
     * 
     */
    virtual void Do() = 0;

    /**
     * @brief The exit action which will executed when the context transitions off of the state.
     * 
     */
    virtual void Exit() = 0;

    /**
     * @brief The HandleEvent function defines how the context will react to the next event, this depends on the state it is currently in.
     * 
     * @param rEvent The next event to be handle by the state machine
     * @param rContext The context which will go through a state because of the incoming event.
     */
    virtual void HandleEvent(CEvent& rEvent, CRobotContext& rContext) = 0;
protected:
private:
};

#endif /*IROBOTSTATES_H*/