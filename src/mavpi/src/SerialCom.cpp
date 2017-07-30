#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <asm/termios.h>
#include "SerialCom.hpp"

//using namespace std;

struct termios Opt;

flSerialPort::flSerialPort(char * portName,int bandRate)
{
        fd = open(portName, O_RDWR | O_NOCTTY);
        if(fd < 0) {
                perror(portName);
                throw "open dev err!";
        }
}

bool flSerialPort::flwrite(string str)
{
        //todo: str2buf
        len = write(fd, buf, sizeof(buf)); /* 向串口写入字符串 */
        if (len < 0) {
                throw "write data error";
        }
}

bool flSerialPort::flread()
{
        char buff[30];
        len = read(fd, buff, sizeof(buff)); /* 在串口读入字符串 */
        if (len < 0) {
                throw "read error";
                //return -1;
                return false;
        }
        if(len > 0)
        {
                str = string(buff);
                return true;
                //printf("%d : %s\n",len, buf); /* 打印从串口读出的字符串 */
        }
        return false;
}
