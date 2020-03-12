/**
 * @file CCommunicate.h
 * @author Tim Beeren (T.Beeren1@student.han.nl)
 * @brief The communicate class makes use of boost asio messaging to write messages to the robotarm through the serial output
 * @version 0.1
 * @date 06-03-2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef CCOMMUNICATE_H
#define CCOMMUNICATE_H

#include <string>
#include <boost/asio.hpp>

class CCommunicate
{
public:
    CCommunicate();
    ~CCommunicate();

    /**
     * @brief Initialises the serial communication
     * 
     * @return true when serial communication has been established and the serial port has been opened
     * @return false when the serial port was unable to be opened for communication
     */
    bool Init();
    /**
     * @brief Closes the serial port if it was open
     * 
     * @return true Returns true when the serial port was succesfully closed
     * @return false Returns false when the serial port could not be closed
     */
    bool Close();
    /**
     * @brief Writes a string message to the robot arm through serial communication
     * 
     * @param rMessage the message to send to the robotarm
     * @return true when the message was successfully sent
     * @return false when the message could not be send
     */
    bool WriteSerial(const std::string& rMessage);

private:
    boost::asio::io_service m_ioService;
    std::string m_serialName;
    boost::asio::serial_port m_serialPort;

};

#endif /*CCOMMUNICATE_H*/
