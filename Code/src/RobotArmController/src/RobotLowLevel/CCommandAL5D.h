/**
 * @file CCommandAL5D.h
 * @author Tim Beeren (T.Beeren1@student.han.nl)
 * @brief 
 * @version 0.1
 * @date 06-03-2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef CCOMMANDAL5D_H
#define CCOMMANDAL5D_H

#include "IExecuteCommand.h"
#include <memory>
#include <vector>
#include <map>
#include <array>

// Forward Declaration
class CCommunicate;

namespace
{
    constexpr const uint8_t INSTRUCTION_ARRAY_SIZE = 5;
    constexpr const char* STOP_MESSAGE = "XSTOP";
}

class CCommandAL5D : public IExecuteCommand
{
public:
    CCommandAL5D();
    ~CCommandAL5D();

    void Stop() override;
    void Write(const std::string& rMessage) override;
    void AppendInstruction(eCommand eCommand, uint64_t position, uint64_t speed, uint64_t duration) override;

    void Execute();

private:
    std::string CreateMessage();

    std::array<uint64_t, INSTRUCTION_ARRAY_SIZE> m_positionArray;
    std::array<uint64_t, INSTRUCTION_ARRAY_SIZE> m_speedArray;
    std::array<uint64_t, INSTRUCTION_ARRAY_SIZE> m_durationArray;

    std::shared_ptr<CCommunicate> m_spCommunicate;
    std::map<eCommand, std::vector<uint64_t>> m_instructionList;
};

#endif /*CCOMMANDAL5D_H*/