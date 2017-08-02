
#include "MsgIo.h"
//#include "protocol.h"
#include <string>
#include <ros/ros.h>



using namespace std;

void MsgIo::sendStatusUseSerial(geometry_msgs::Vector3& eular,
                                mavros_msgs::State& current_state,
                                geometry_msgs::PoseStamped& pos_status,
                                sensor_msgs::Range& dis_status
                                )
{
        download_s dp = makeDownPack(
                (current_state.armed) ? Arm : NoArm,
                (current_state.mode == "OFFBOARD") ? Stable : OffBoard,
                pos_status.pose.position.x, pos_status.pose.position.y,
                pos_status.pose.position.z,
                eular.z,
                Data.setPos.x,
                Data.setPos.y,
                Data.setPos.z,
                Data.setPos.w,
                eular.x,
                eular.y,
                0, //
                0, //
                dis_status.range);
        char* dt = packDownload(&dp);
        ROS_INFO("%s",dt);
        ROS_INFO("[%d]",s.Write(dt, strlen(dt)));
}

void MsgIo::checkUp(char* s)
{
        upload_s upc;
        if (unpackUp(s, &upc)) {
                Data.ctrlMode = (enum CtrlMode)upc.ctrlMode;

                Data.setPos.x = (float)upc.x / upc.div;
                Data.setPos.y = (float)upc.y / upc.div;
                Data.setPos.z = (float)upc.z / upc.div;
                Data.setPos.w = (float)upc.yaw / upc.div;

                Data.pitch = (float)upc.pitch / upc.div;
                Data.roll  = (float)upc.roll / upc.div;
                // check data
                // r,0,0,0,150,70,100,20,
                switch (Data.ctrlMode) {
                case Pose:
                        ROS_WARN("REC CTRL CMD => x: [%f], y: [%f], z: [%f], yaw: [%f]", Data.setPos.x, Data.setPos.y, Data.setPos.z, Data.setPos.w);
                        break;

                case Raw:
                        ROS_WARN("REC CTRL CMD => r: [%f], p: [%f], yaw: [%f], th: [%f]", Data.roll, Data.pitch, Data.setPos.w, Data.setPos.z);
                        break;

                default:
                        break;
                }
        } else {
                ROS_ERROR("REC CMD ERR!");
        }
}

void MsgIo::SerialCallBack(string str)
{
        ROS_INFO("%s",str.c_str());
        char temp[BufferLength];
        strcpy(temp, str.c_str());
        checkUp(temp);
}

void MsgIo::checkSerialCmd()
{
        char buffer[BUFFER_SIZE];
        string buff   = "";
        bool save   = false;

        // check serial data
        int bLength = s.Read(buffer);

        for (int i = 0; i < bLength; i++) {
                if (readFilter(buffer + i)) {
                        if (buffer[i] == 'r' && !save) {
                                buff.clear();
                                save = true;
                        } else if (buffer[i] == '\n' && save) {
                                buff.push_back(buffer[i]);
                                SerialCallBack(buff);
                                save = false;
                                buff.clear();
                                s.clear();
                        }

                        if (save) {
                                buff.push_back(buffer[i]);
                        }
                }
        }
}

void MsgIo::clear()
{
        s.clear();
}
