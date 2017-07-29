#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

using namespace std;
using namespace boost::asio;

typedef string any_type;


class MySerialPort
{
private:
    //Initialize port
    bool init_port( const any_type port, const unsigned int char_size );
public:
    //Constructor
    //Destructor
        MySerialPort( const any_type &port_name );
    ~MySerialPort();

    //Write some data to port
    void write_to_serial( const any_type data );

    //Read data from port which write data just now
    void read_from_serial();

    //The asyanc callback function of asyanc_read
    void handle_read( char buf[], boost::system::error_code ec,
        std::size_t bytes_transferred );

    //To Call io_service::run function to call asyanc callback funciton
    void call_handle();

private:
    //io_service Object
    io_service m_ios;

    //Serial port Object
    serial_port *pSerialPort;

    //For save com name
    any_type m_port;

    //Serial_port function exception
    boost::system::error_code ec;
};
