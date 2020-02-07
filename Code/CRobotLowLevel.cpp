#include "CRobotLowLevel.h"
#include <iostream>
#include <boost/asio.hpp>
#include <thread>

CRobotLowLevel::CRobotLowLevel(/* args */)
{
}

CRobotLowLevel::~CRobotLowLevel()
{
}

bool CRobotLowLevel::Move(uint16_t pwm, uint64_t time)
{
    if (IsHardwareCompatible(pwm))
    {
        WriteToSerialPort(ToProtocolMessage(pwm, time));
    }
}

uint16_t CRobotLowLevel::GetMaxPWM()
{
    return m_maxPWM;
}

std::string CRobotLowLevel::ToProtocolMessage(uint16_t pwm, uint64_t time)
{
    std::string message = "$";
    message.append(std::to_string(pwm));
    message.append(":");
    message.append(std::to_string(time));
    message.append("!");
    return message;
}

bool CRobotLowLevel::IsHardwareCompatible(uint16_t pwm)
{
    return ((pwm <= m_maxPWM) && (pwm > 0));
}

bool CRobotLowLevel::WriteToSerialPort(const std::string &message)
{
    boost::asio::io_service ioservice;
	boost::asio::serial_port serial(ioservice, m_serialPort);

	serial.set_option(boost::asio::serial_port_base::baud_rate(9600));
	serial.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
	serial.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
	serial.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
	serial.set_option(boost::asio::serial_port::character_size(boost::asio::serial_port::character_size(8)));

	for (const char& s: message)
	{

	    boost::asio::streambuf b;
	    std::ostream os(&b);
	    os << s << "\r";
	    boost::asio::write(serial, b.data());
	    os.flush();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

     	}

        if (serial.is_open()) {
		serial.close();
	}

    return true;
}