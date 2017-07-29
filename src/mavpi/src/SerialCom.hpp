#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <asm/termios.h>
//#define DEV_NAME "/dev/ttyUSB1"
#include <iostream>

using namespace std;

typedef string any_type;


class flSerialPort
{
//////////////////////////////
public:
flSerialPort(char * portName,
             int bandRate);

bool flwrite(string str);

bool flread();

string str;

/////////////////////////////
private:
int fd;
int len, i,ret;
char buf[30];

};
