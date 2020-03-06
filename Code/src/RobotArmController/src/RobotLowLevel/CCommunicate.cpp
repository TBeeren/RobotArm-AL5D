/**
 * @file CCommunicate.cpp
 * @author Tim Beeren (T.Beeren1@student.han.nl)
 * @brief 
 * @version 0.1
 * @date 06-03-2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "CCommunicate.h"
#include <iostream>

CCommunicate::CCommunicate()
: m_serialName("/dev/ttyACM0")
, m_serialPort(m_ioService, m_serialName)
{
}

CCommunicate::~CCommunicate()
{
}

bool CCommunicate::Init()
{
	m_serialPort.set_option(boost::asio::serial_port_base::baud_rate(9600));
	m_serialPort.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
	m_serialPort.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
	m_serialPort.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
	m_serialPort.set_option(boost::asio::serial_port::character_size(boost::asio::serial_port::character_size(8)));

    return m_serialPort.is_open();
}

bool CCommunicate::WriteSerial(const std::string& rMessage)
{
	for (const char& s: rMessage)
	{
	    boost::asio::streambuf b;
	    std::ostream os(&b);
	    os << s << "\r";
        
	    boost::asio::write(m_serialPort, b.data());
	    os.flush();
    }

    if (m_serialPort.is_open()) 
    {
		m_serialPort.close();
	}

    return true;
}