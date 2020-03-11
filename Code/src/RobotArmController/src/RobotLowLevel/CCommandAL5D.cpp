/**
 * @file CCommandAL5D.cpp
 * @author Tim Beeren (T.Beeren1@student.han.nl)
 * @brief 
 * @version 0.1
 * @date 06-03-2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "CCommandAL5D.h"

#include "CCommunicate.h"

namespace
{
    constexpr const int16_t MINIMAL_DEGREES_BASE = 500;
    constexpr const int16_t MINIMAL_DEGREES_SHOULDER = 500;
    constexpr const int16_t MINIMAL_DEGREES_ELBOW = 500;
    constexpr const int16_t MINIMAL_DEGREES_WRIST = 500;
    constexpr const int16_t MINIMAL_DEGREES_WRIST_ROTATE = 500;

    constexpr const int16_t MAXIMAL_DEGREES_BASE = 2500;
    constexpr const int16_t MAXIMAL_DEGREES_SHOULDER = 2500;
    constexpr const int16_t MAXIMAL_DEGREES_ELBOW = 2500;
    constexpr const int16_t MAXIMAL_DEGREES_WRIST = 2500;
    constexpr const int16_t MAXIMAL_DEGREES_WRIST_ROTATE = 2500;
}

CCommandAL5D::CCommandAL5D()
: m_spCommunicate(std::make_shared<CCommunicate>())
{
}

CCommandAL5D::~CCommandAL5D()
{
}

void CCommandAL5D::Write(const std::string& rMessage)
{
    m_spCommunicate->WriteSerial(rMessage);
}

void CCommandAL5D::ClearLists()
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    m_targets.clear();
    m_positions.clear();
    m_speeds.clear();
    m_durations.clear();
}

void CCommandAL5D::AppendInstruction(eCommand eCommand, int8_t servo ,int64_t position, int64_t speed, int64_t duration)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;


    switch (eCommand)
    {
        case eCommand::MOVE_COMMAND:
        {
            m_targets.push_back(servo);
            m_positions.push_back(position);
            m_speeds.push_back(speed);
            m_durations.push_back(duration);
            break;
        }
        case eCommand::STOP_COMMAND:
        {
            Stop();
            break;
        }
        case eCommand::UNKNOWN_COMMAND:
        {
            std::cout<< "Command is unknown! Please check your message on syntax" << std::endl;
            break;
        }
        default:
        {
            break;
        }
    }
}

bool CCommandAL5D::IsHardwareCompatible(eServos servo, int64_t position)
{
    bool success = false;

    switch (servo)
    {
        case eServos::BASE:
        {
            success = (position >= MINIMAL_DEGREES_BASE && position <= MAXIMAL_DEGREES_BASE);
            break;
        }
        case eServos::SHOULDER:
        {
            success = (position >= MINIMAL_DEGREES_SHOULDER && position <= MAXIMAL_DEGREES_SHOULDER);
            break;
        }
        case eServos::ELBOW:
        {
            success = (position >= MINIMAL_DEGREES_ELBOW && position <= MAXIMAL_DEGREES_ELBOW);
            break;
        }
        case eServos::WRIST:
        {
            success = (position >= MINIMAL_DEGREES_WRIST && position <= MAXIMAL_DEGREES_WRIST);
            break;
        }
        case eServos::WRIST_ROTATE:
        {
            success = (position >= MINIMAL_DEGREES_WRIST_ROTATE && position <= MAXIMAL_DEGREES_WRIST_ROTATE);
            break;
        }
        default:
        {
            std::cout<< "Servo is unknown! Please check your message on syntax" << std::endl;
            break;
        }
    }
    
    return success;
}

// Servo ID: #5 
// Position: P1600 
// Time    : T2500 
// Speed   : S500
std::string CCommandAL5D::CreateMessage()
{
    std::string returnMessage = "";

    for(uint16_t i = 0; i < m_targets.size(); ++i)
    {
        returnMessage.append("#");
        returnMessage.append(std::to_string(m_targets.at(i)));

        if(!IsHardwareCompatible(eServos(i), m_positions.at(i)))
        {
            break;
        }

        if(m_positions.at(i) != -1)
        {
            returnMessage.append("P");
            returnMessage.append(std::to_string(m_positions.at(i)));
        }

        if(m_durations.at(i) != -1)
        {
            returnMessage.append("T");
            returnMessage.append(std::to_string(m_durations.at(i)));
        }

        if(m_speeds.at(i) != -1)
        {
            returnMessage.append("S");
            returnMessage.append(std::to_string(m_speeds.at(i)));
        }

    }
    char cr = 13;
    returnMessage += cr;

    if(returnMessage.size() < 4)
    {
        std::cout<< "[Error]: Created message doesn't suite the protocol."<<std::endl;
    }

    ClearLists();
    std::cout << returnMessage << std::endl;
    
    return returnMessage;
}

void CCommandAL5D::Execute()
{
    // Assert if the instruction arrays aren't equal and bigger then 0. 
    assert(m_targets.size() != 0);
    assert(m_positions.size() == m_speeds.size() == m_durations.size());

    // Write Serial with the converted string.
    m_spCommunicate->WriteSerial(CreateMessage());
}   

void CCommandAL5D::Stop()
{
    m_spCommunicate->WriteSerial(STOP_MESSAGE);
}
