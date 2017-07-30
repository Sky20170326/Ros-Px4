
#include <string>
#include <iostream>
using namespace std;


#include "../SerialCom.hpp"
#include "../protocol.h"

void error(string msg)
{
        cerr << "error => " << msg << endl;
}

void info(string msg)
{
        cout << "info => " << msg << endl;
}

void serialtest()
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
}

void checkUp(char * s)
{
    upload_s upc;
    info("check string : " + string(s));
    if( unpackUp (s,&upc))
        info("uppack check ok");
    else
        error("uppack check error!");
}

int main (int argc, char *argv[])
{
        char teststr[] = "rs0123456789\n";
        if(CheckString(teststr))
        {
                info("CheckString Ok");
        }
        else
        {
                error("CheckString err");
        }

        upload_s up = makeUpPack(0.0,0.0,1.5,0.7);
        info("up make!");
        char * ut = packUpload(&up);
        info("up: " + string(ut));


        download_s dp = makeDownPack(0.2,0.3,0.5,0.2,0.0,0.0,1.5,0.7);
        char * dt = packDownload(&dp);
        info("down" + string(dt));


        char utc[] = "r,0,0,0,150,70,100,20,\n";
        checkUp(utc);
        char utcerr1[] = "r,0,0,0,150,70,100,20,50,60,\n";
        checkUp(utcerr1);
        char utcerr2[] = "r,0,0,0,150,70,10,\n";
        checkUp(utcerr2);
        char utcerr3[] = "0,0,0,150,70,10,\n";
        checkUp(utcerr3);
        char utcerr4[] = "r,0,0,0,150,70,10";
        checkUp(utcerr4);





        return(0);
}
