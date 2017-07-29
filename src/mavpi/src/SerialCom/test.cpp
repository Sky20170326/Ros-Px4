
#include <string>
#include <iostream>
using namespace std;


#include "../SerialCom.hpp"

void error(char * msg)
{
        cout << "error => " << msg << endl;
}

int main (int argc, char *argv[])
{
        try{

                flSerialPort serial((char*)"/dev/ttyUSB1",9600);

                for (int i = 0; i < 255; i++)
                {
                        serial.flwrite("a");
                }

                while(1)
                {
                        if(serial.flread())
                        {
                                cout << serial.str.length() << " : "
                                     << serial.str << endl;
                        }
                }
        }
        catch( char * e )
        {
                error(e);
        }

        return(0);
}
