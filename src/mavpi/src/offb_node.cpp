/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
//#include <sensor_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
//#include <boost/asio.hpp>
////////////////////////////////////////
#include "serial.h"
#include "protocol.h"

using namespace std;

//#define simulator

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
}

sensor_msgs::Imu imu_status;
void imu_state_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
        //ROS_INFO("Imu Seq: [%d]", msg->header.seq);
        imu_status = *msg;
}

geometry_msgs::PoseStamped pos_status;
void pose_state_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
        //ROS_INFO("test");
        pos_status = *msg;
}

sensor_msgs::Range dis_status;
void dis_state_cb(const sensor_msgs::Range::ConstPtr& msg)
{
        //ROS_INFO("test");
        dis_status = *msg;
}

geometry_msgs::PoseStamped xyz2Position (float x,float y,float z)
{
        geometry_msgs::PoseStamped pose;

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;

        return pose;
}

void quad(float q0,float q1,float q2,float q3,float *pitch,float *roll,float *yaw)
{
        *pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
        *roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
        *yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;   //yaw
}

serial serial;
float rec_x = 0,
      rec_y = 0,
      rec_z = 0,
      rec_yaw = 0;
char buffer[BUFFER_SIZE];
string buff = "";
bool save = false;
int length = 0;

void checkUp(char * s)
{
        upload_s upc;
        if( unpackUp (s,&upc))
        {
                rec_x = (float)upc.x / upc.div;
                rec_y = (float)upc.y / upc.div;
                rec_z = (float)upc.z / upc.div;
                rec_yaw = (float)upc.yaw / upc.div;
                //check data
                // r,0,0,0,150,70,100,20,
                ROS_INFO("REC CTRL CMD => x: [%f], y: [%f], z: [%f], yaw: [%f]",
                         rec_x,
                         rec_y,
                         rec_z,
                         rec_yaw
                         );
        }
        else
        {
                ROS_WARN("REC CMD ERR!");
        }
}

void SerialCallBack(string str)
{
        ROS_INFO(str.c_str());
        char temp [BufferLength];
        strcpy(temp,str.c_str());
        checkUp(temp);
}

int main(int argc, char **argv)
{
        ROS_INFO("System init ...");

        serial.Open("/dev/ttyUSB0", 115200, 8, NO, 1);

        ros::init(argc, argv, "offb_node");
        ros::NodeHandle nh;

        ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                            ("mavros/state", 10, state_cb);

        ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
                                          ("mavros/imu/data", 10, imu_state_cb);

        ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
                                          ("mavros/local_position/pose", 10, pose_state_cb);

        ros::Subscriber dis_sub = nh.subscribe<sensor_msgs::Range>
                                          ("mavros/px4flow/ground_distance", 10, dis_state_cb);

        ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                               ("mavros/setpoint_position/local", 10);

        //ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
        //    ("mavros/setpoint_raw/local",10);

        ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                                   ("mavros/cmd/arming");

        ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                                     ("mavros/set_mode");

        //the setpoint publishing rate MUST be faster than 2Hz
        ros::Rate rate(20.0);

        ROS_INFO("connecting with mavros ...");
        // wait for FCU connection
        while(ros::ok() && current_state.connected) {
                cout << "wait connect" << endl;
                ros::spinOnce();
                rate.sleep();
        }
        ROS_INFO("connected!");

        ROS_INFO("pre offboard...");
        //send a few setpoints before starting
        for(int i = 100; ros::ok() && i > 0; --i) {
                local_pos_pub.publish(xyz2Position(0,0,0));
                ros::spinOnce();
                rate.sleep();
        }

        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        ros::Time last_request = ros::Time::now();

        serial.clear();

        ROS_INFO("pre fished!");
        
        while(ros::ok()) {

#ifdef simulator
                //turn board into offboard mode
                if( current_state.mode != "OFFBOARD" &&
                    (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                        if( set_mode_client.call(offb_set_mode) &&
                            offb_set_mode.response.success)
                        {
                                ROS_INFO("Offboard enabled");
                        }
                        last_request = ros::Time::now();
                }
                else
                {
                        //check system armd
                        if( !current_state.armed &&
                            (ros::Time::now() - last_request > ros::Duration(5.0)))
                        {
                                if( arming_client.call(arm_cmd) &&
                                    arm_cmd.response.success)
                                {
                                        ROS_INFO("Vehicle armed");
                                }
                                last_request = ros::Time::now();
                        }
                }

#endif


                //check serial data
                int bLength = serial.Read(buffer);

                for (int i = 0; i < bLength; i++)
                {
                        if(readFilter(buffer + i))
                        {
                                if(buffer[i] == 'r' && !save)
                                {
                                        buff.clear();
                                        save = true;
                                }
                                else if(buffer[i] == '\n' && save)
                                {
                                        buff.push_back(buffer[i]);
                                        SerialCallBack(buff);
                                        save = false;
                                        buff.clear();
                                        serial.clear();
                                }

                                if(save)
                                {
                                        buff.push_back(buffer[i]);
                                }
                        }
                }


                //set status led

                //pubpos
                local_pos_pub.publish(xyz2Position (0,0,1.5));


                //calc eular
                float a,b,c;
                quad(imu_status.orientation.w,
                     imu_status.orientation.x,
                     imu_status.orientation.y,
                     imu_status.orientation.z,
                     &a, &b, &c);
                //pub to mcu
                //cout arm & mode

                cout << (current_state.armed == true) ? "0" : "1";
                cout << '\t' << current_state.mode << endl;
                //show imu
                cout << ros::Time::now() << '\t' << endl;

                ROS_INFO("Euler => x: [%f], y: [%f], z: [%f]",a,b,c);


                ROS_INFO("POSE => x: [%f], y: [%f], z: [%f]",
                         pos_status.pose.position.x,
                         pos_status.pose.position.y,
                         pos_status.pose.position.z
                         );

                ROS_INFO("DIS => d: [%f]",dis_status.range);

                download_s dp = makeDownPack(pos_status.pose.position.x,pos_status.pose.position.y,pos_status.pose.position.z,c,
                                             0,0,0,0,
                                             a,b,dis_status.range);
                char * dt = packDownload(&dp);

                ROS_INFO(dt);
                serial.Write(dt, strlen(dt));

                ros::spinOnce();
                rate.sleep();
        }

        return 0;
} /**
   * @file offb_node.cpp
   * @brief offboard example node, written with mavros version 0.14.2, px4 flight
   * stack and tested in Gazebo SITL
   */
