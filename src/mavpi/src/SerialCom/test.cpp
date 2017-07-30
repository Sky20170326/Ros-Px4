
#include <string>
#include <iostream>
using namespace std;
//#include "../SerialCom.hpp"
#include "../protocol.h"
#include "../serial.h"

void error(string msg)
{
        cerr << "error => " << msg << endl;
}

void info(string msg)
{
        cout << "info => " << msg << endl;
}

void SerialHandle(string buf)
{
        info("receive: " + buf);
}

serial serial;
void serialwritetest()
{
    serial.Open("/dev/ttyUSB0", 9600, 8, NO, 1);

    serial.Write("aaaaa", 5);

    serial.Close();
}

void serialReadTest()
{
    serial.Open("/dev/ttyUSB0", 9600, 8, NO, 1);

    char buffer[BufferLength] = {0};
    int length = 0;
    while(1)
	{
		// Wait character
		length = serial.Read(buffer);

		if(length)
		{
			for(int i = 0; i < length; i++)
			{
				printf("%.2X ", buffer[i]);
				//printf("%c", buffer[i]);
			}
            printf("\n");
		}
	}

    serial.Close();
}

void serialtest()
{
    serialwritetest();
    serialReadTest();
}

void checkUp(char * s)
{
        upload_s upc;
        info("check up string : " + string(s));
        if( unpackUp (s,&upc))
                info("uppack check ok");
        else
                error("uppack check error!");
}

void checkDown(char * s)
{
        download_s dpc;
        info("check down string : " + string(s));
        if( unpackDown(s,&dpc))
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


        download_s dp = makeDownPack(0.2,0.3,0.5,0.2,0.0,0.0,1.5,0.7,0.2,0.5,0.3);
        char * dt = packDownload(&dp);
        info("down: " + string(dt));

        //upload unpack test
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

        //unpack download test
        char dtc[] = "s,0,20,30,50,20,0,0,150,70,20,50,30,100,40,\n";
        checkDown(dtc);
        char dtcerr1[] = "s,0,20,30,50,20,0,0,15100,40,\n";
        checkDown(dtcerr1);
        char dtcerr2[] = ",0,20,30,50,20,0,0,150,70,100,40,\n";
        checkDown(dtcerr2);
        char dtcerr3[] = "s,0,20,30,50,20,0,0,150,70,100,40,";
        checkDown(dtcerr3);
        char dtcerr4[] = "s,0,20,30,50,20,0,0,150,70,10,100,40,\n";
        checkDown(dtcerr4);

        //serialtest();

        return(0);
}
