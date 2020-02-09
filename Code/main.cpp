#include "CRobotHighLevel.h"
#include <iostream>

int main(int argc, char const *argv[])
{
    if (argc != 4)
    {
        std::cout << "[Warning] -> Please enter three arguments in the given format:" << std::endl;
        std::cout << "      [1] -> position";
        std::cout << "      [2] -> goniotype as 0 for degrees or 1 for radians";
        std::cout << "      [3] -> milliseconds";
    }
    else
    {
        CRobotHighLevel robot;
        robot.Move(std::stoi(argv[1]), eGonioType(std::stoi(argv[2])), std::stoi(argv[3]));
        std::cout << "Arm moving to: " << argv[1] << " , " << argv[2] << " in: " << argv[3] << " milliseconds" << std::endl;
    }
    return 0;
}
