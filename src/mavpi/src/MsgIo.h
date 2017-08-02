#ifndef _MSG_IO_
#define _MSG_IO_

#include "serial.h"
#include "protocol.h"
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Range.h>

class MsgIo {
private:
        serial s;
        char* _portName;


public:

        struct RecData {
                geometry_msgs::Quaternion setPos;
                enum CtrlMode ctrlMode;
                float pitch = 0, roll = 0;
        } Data;

        MsgIo();
        MsgIo(char * portName)
        {
                _portName = portName;
                s.Open(_portName, 115200, 8, NO, 1);
        }

        void checkSerialCmd();
        void checkUp(char* s);
        void SerialCallBack(std::string str);
        void sendStatusUseSerial(geometry_msgs::Vector3 &eular,
                                 mavros_msgs::State &current_state,
                                 geometry_msgs::PoseStamped& pos_status,
                                 sensor_msgs::Range& dis_status
                                 );
        void clear();

        // ~MsgIo()                  = default;
        // MsgIo(const MsgIo& other) = default;
        // MsgIo(MsgIo&& other)      = default;
        // MsgIo& operator=(const MsgIo& other) = default;
        // MsgIo& operator=(MsgIo&& other) = default;

};

#endif //_MSG_IO_
