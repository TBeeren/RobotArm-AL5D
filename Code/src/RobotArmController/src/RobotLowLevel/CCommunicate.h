/**
 * @file CCommunicate.h
 * @author Tim Beeren (T.Beeren1@student.han.nl)
 * @brief 
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

    bool Init();
    bool WriteSerial(const std::string& rMessage);

private:
    std::string m_serialName;
    boost::asio::serial_port m_serialPort;
    boost::asio::io_service m_ioService;

};

#endif /*CCOMMUNICATE_H*/
