#include <ros/ros.h>
#include "RobotArmController/Move.h"
#include "RobotArmController/ServoCommand.h"
#include "RobotArmController/EmergencyStop.h"
#include "RobotArmController/ProgrammedPosition.h"

#include <iostream>
#include <algorithm>
#include <vector>

int main(int argc, char *argv[])
{
    /* after calibrate the protocol == #2P2000D500S400 */
    ros::init(argc, argv, "MotionController");
    ros::NodeHandle node;
    ros::Publisher calibratePublisher = node.advertise<RobotArmController::Move>("/RobotArmController/Calibrate", 1000);
    ros::Publisher movePublisher = node.advertise<RobotArmController::Move>("/RobotArmController/Move", 1000);
    ros::Publisher programmedPositionPublisher = node.advertise<RobotArmController::ProgrammedPosition>("/RobotArmController/ProgrammedPosition", 1000);
    ros::Publisher stopPublisher = node.advertise<RobotArmController::EmergencyStop>("/RobotArmController/EmergencyStop", 1000);

    while (!ros::ok())
        ;

    std::string input = "";

    //Calibration
    std::cout << "Calibrate using wasd and enter, type done when completed" << std::endl;
    int calibratingServo = 0;
    std::vector<int> calibrationValues(5, 100);
    while (ros::ok())
    {
        std::cin >> input;
        std::cout << "Input is: " << input << std::endl;
        if (input == "w")
        {
            calibrationValues.at(calibratingServo) += 10;
        }
        if (input == "a")
        {
            --calibratingServo;
        }
        if (input == "s")
        {
            calibrationValues.at(calibratingServo) -= 10;
        }
        if (input == "d")
        {
            ++calibratingServo;
        }
        if (input == "done")
        {
            break;
        }
        std::cout << "For Servo: " << calibratingServo << " setting value: " << calibrationValues.at(calibratingServo) << std::endl;

        if (input == "w" || input == "s")
        {
            std::vector<RobotArmController::ServoCommand> servoCommands;
            RobotArmController::ServoCommand servoCommand;
            RobotArmController::Move calibrationMsg;
            servoCommand.targetServo = calibratingServo;
            servoCommand.position = calibrationValues.at(calibratingServo);
            servoCommand.speed = 0;
            servoCommand.duration = 0;
            servoCommands.push_back(servoCommand);
            calibrationMsg.instruction = servoCommands;
            calibrationMsg.preemptive = false;
            calibratePublisher.publish(calibrationMsg);
        }
        ros::spinOnce();
    }
    //End of Calibration

    while (ros::ok())
    {
        std::cin >> input;
        if (input == "exit")
        {
            break;
        }
        else
        {
            std::cout << "Input: " << input << std::endl;
            //Emergency Stop

            //ProgrammedPositions

            //Controlling Servos
            size_t instructionAmount = std::count(input.begin(), input.end(), '#');
            if (instructionAmount != 0)
            {
                std::vector<RobotArmController::ServoCommand> servoCommands;

                std::size_t commandStart = 0;
                std::string command = "";
                //queue#1P2000S300D400!#5P600S670D890!

                //Create the message
                RobotArmController::Move moveMsg;

                //Check if preemptive
                moveMsg.preemptive = true;
                if (input.find("queue") != std::string::npos)
                {
                    std::cout << "Queueing command:" << std::endl;
                    moveMsg.preemptive = false;
                }

                //Parse Instructions
                for (int i = 0; i < instructionAmount; ++i)
                {
                    command = input.substr(commandStart, input.find("!"));
                    input.erase(0, input.find("!") + 1);

                    std::size_t servoStart = command.find("#");
                    std::size_t positionStart = command.find("P");
                    std::size_t speedStart = command.find("S");
                    std::size_t durationStart = command.find("D");

                    std::string servoString, positionString, durationString, speedString = "";
                    for (int i = servoStart + 1; isdigit(command[i]); ++i)
                    {
                        servoString += command[i];
                    }
                    for (int i = positionStart + 1; isdigit(command[i]); ++i)
                    {
                        positionString += command[i];
                    }
                    for (int i = speedStart + 1; isdigit(command[i]); ++i)
                    {
                        speedString += command[i];
                    }
                    for (int i = durationStart + 1; isdigit(command[i]); ++i)
                    {
                        durationString += command[i];
                    }
                    std::cout << "Moving servo: " << servoString << " to position: " << positionString << " with max speed: " << speedString << " in " << durationString << " milliseconds?" << std::endl;

                    RobotArmController::ServoCommand servoCommand;
                    servoCommand.targetServo = stoi(servoString);
                    servoCommand.position = stoull(positionString);
                    servoCommand.speed = stoull(speedString);
                    servoCommand.duration = stoull(durationString);
                    servoCommands.push_back(servoCommand);
                }
                moveMsg.instruction = servoCommands;
                movePublisher.publish(moveMsg);
            }
        }
        ros::spinOnce();
    }
    //for(int i = 0; )
    return 0;
}
