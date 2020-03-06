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

CCommandAL5D::CCommandAL5D()
: m_spCommunicate(std::make_shared<CCommunicate>())
, m_positionArray({0})
, m_speedArray({0})
, m_durationArray({0})
{
}

CCommandAL5D::~CCommandAL5D()
{
}

void CCommandAL5D::Write(const std::string& rMessage)
{
    m_spCommunicate->WriteSerial(rMessage);
}

void CCommandAL5D::AppendInstruction(eCommand eCommand, uint64_t position, uint64_t speed, uint64_t duration)
{
    switch (eCommand)
    {
        case eCommand::MOVE_COMMAND:
        {
            m_positionArray.back() = position;
            m_speedArray.back() = speed;
            m_durationArray.back()= duration;
            break;
        }
        case eCommand::STOP_COMMAND:
        {
            Stop();
            break;
        }
        case eCommand::UNKNOWN_COMMAND:
        {
            throw "Command is unknown! Please check your message on syntax";
            break;
        }
        default:
        {
            break;
        }
    }
}

// Servo ID: #5 
// Position: P1600 
// Time    : T2500 
// Speed   : S500
std::string CCommandAL5D::CreateMessage()
{
    std::string returnMessage = "";

    for(int i = 0; i < m_positionArray.size(); ++i)
    {
        returnMessage += "#" + i;

        if(m_positionArray[i] != -1)
        {
            returnMessage += "P" + m_positionArray[i];
        }

        if(m_durationArray[i] != -1)
        {
            returnMessage += "T" + m_durationArray[i];
        }

        if(m_speedArray[i] != -1)
        {
            returnMessage += "S" + m_speedArray[i];
        }
    }

    if(returnMessage.size() < 4)
    {
        throw "[Error]: Created message doesn't suite the protocol.";
    }
    else
    {
        std::fill(m_positionArray.begin(), m_positionArray.end(), 0);
        std::fill(m_durationArray.begin(), m_durationArray.end(), 0);
        std::fill(m_speedArray.begin(), m_speedArray.end(), 0);
    }

    return returnMessage;
}

void CCommandAL5D::Execute()
{
    // Assert if the instruction arrays aren't equal and bigger then 0. 
    assert(m_positionArray.size() != 0);
    assert(m_positionArray.size() == m_speedArray.size() == m_durationArray.size());

    // Write Serial with the converted string.
    m_spCommunicate->WriteSerial(CreateMessage());
}

void CCommandAL5D::Stop()
{
    m_spCommunicate->WriteSerial(STOP_MESSAGE);
}
