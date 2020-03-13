/**
 * @file CCommandAL5D.h
 * @author Tim Beeren (T.Beeren1@student.han.nl)
 * @brief The CommandAL5D class is a realistation of the IExecuteCommand. It is reponsible for commanding the AL5D robotarm and is used by higher level drivers
 * @version 0.1
 * @date 06-03-2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef CCOMMANDAL5D_H
#define CCOMMANDAL5D_H

#include "IExecuteCommand.h"
#include "../RobotHighLevel/CConfiguration.h"
#include <memory>
#include <vector>
#include <map>
#include <array>

// Forward Declaration
class CCommunicate;

namespace
{
    constexpr const uint8_t INSTRUCTION_ARRAY_SIZE = 5;
    constexpr const char* STOP_MESSAGE = "STOP";
}

class CCommandAL5D : public IExecuteCommand
{
public:
    CCommandAL5D();
    ~CCommandAL5D();

    void Stop() override;
    void Write(const std::string& rMessage) override;
    void AppendInstruction(eCommand eCommand, int8_t servo ,int64_t position, int64_t speed, int64_t duration) override;
    void ClearLists() override;
    void Execute() override;

private:
    /**
     * @brief Creates a string messages based on instructions appended to this class.
     * 
     * @return std::string the resulting string message to be sent to the AL5D robotarm.
     */
    std::string CreateMessage();
    /**
     * @brief Checks whether the position specified is within the reachable boundries for the robotarm.
     * 
     * @param servo the servo for which to check the validity of the position
     * @param position the position that needs to be checked for hardware compatibility
     * @return true Returns true if command is compatible with the hardware
     * @return false Returns false if the command is not compatible with the hardware
     */
    bool IsHardwareCompatible(eServos servo, int64_t position);


    std::shared_ptr<CCommunicate> m_spCommunicate;

    std::vector<int64_t> m_targets;
    std::vector<int64_t> m_positions;
    std::vector<int64_t> m_speeds;
    std::vector<int64_t> m_durations;
    std::map<eCommand, std::vector<uint64_t>> m_instructionList;
};

#endif /*CCOMMANDAL5D_H*/