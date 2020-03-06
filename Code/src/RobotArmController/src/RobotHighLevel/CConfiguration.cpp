#include "CConfiguration.h"

CConfiguration::CConfiguration(/* args */)
{
    programmedPositions =
    {
        {eProgrammedPosition::PARK, "PARK"},
        {eProgrammedPosition::READY, "READY"},
        {eProgrammedPosition::STRAIGHT, "STRAIGHT"}
    };
}

CConfiguration::~CConfiguration()
{
}

eProgrammedPosition CConfiguration::StringToProgrammedPosition(std::string programmedPositionString)
{
    for(auto programmedPosition : programmedPositions)
    {
        if(programmedPosition.second == programmedPositionString)
        {
            return programmedPosition.first;
        }
    }
}