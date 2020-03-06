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
