/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
//#include <sensor_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
//#include <boost/asio.hpp>
////////////////////////////////////////
#include "protocol.h"
#include "serial.h"

using namespace std;

//#define simulator

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) { current_state = *msg; }

sensor_msgs::Imu imu_status;
void imu_state_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    // ROS_INFO("Imu Seq: [%d]", msg->header.seq);
    imu_status = *msg;
}

geometry_msgs::PoseStamped pos_status;
void pose_state_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // ROS_INFO("test");
    pos_status = *msg;
}

sensor_msgs::Range dis_status;
void dis_state_cb(const sensor_msgs::Range::ConstPtr& msg)
{
    // ROS_INFO("test");
    dis_status = *msg;
}

geometry_msgs::PoseStamped xyz2Position(float x, float y, float z)
{
    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    return pose;
}

//四元数转欧拉角
geometry_msgs::Vector3 quad2eular(geometry_msgs::Quaternion quad)
{
    geometry_msgs::Vector3 eular;

    eular.x = asin(-2 * quad.x * quad.z + 2 * quad.w * quad.y) * 57.3; // pitch
    eular.y = atan2(2 * quad.y * quad.z + 2 * quad.w * quad.x,
                  -2 * quad.x * quad.x - 2 * quad.y * quad.y + 1)
        * 57.3; // roll
    eular.z = atan2(2 * (quad.x * quad.y + quad.w * quad.z),
                  quad.w * quad.w + quad.x * quad.x - quad.y * quad.y - quad.z * quad.z)
        * 57.3; // yaw

    return eular;
}

serial serial;
geometry_msgs::Quaternion setPos;
void checkUp(char* s)
{
    upload_s upc;
    if (unpackUp(s, &upc)) {
        setPos.x = (float)upc.x / upc.div;
        setPos.y = (float)upc.y / upc.div;
        setPos.z = (float)upc.z / upc.div;
        setPos.w = (float)upc.yaw / upc.div;
        // check data
        // r,0,0,0,150,70,100,20,
        ROS_INFO("REC CTRL CMD => x: [%f], y: [%f], z: [%f], yaw: [%f]",
            setPos.x, setPos.y, setPos.z, setPos.w);
    } else {
        ROS_WARN("REC CMD ERR!");
    }
}

void SerialCallBack(string str)
{
    ROS_INFO(str.c_str());
    char temp[BufferLength];
    strcpy(temp, str.c_str());
    checkUp(temp);
}

void waitRosConnect(ros::Rate& rate)
{
    ROS_INFO("connecting with mavros ...");
    while (ros::ok() && current_state.connected) {
        cout << "wait connect" << endl;
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("connected!");
}

void preOffboard(ros::Rate& rate, ros::Publisher& local_pos_pub)
{
    ROS_INFO("pre offboard...");
    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(xyz2Position(0, 0, 0));
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("pre offboard fished!");
}

void sendStatusUseSerial(geometry_msgs::Vector3 eular)
{
    download_s dp = makeDownPack(pos_status.pose.position.x, pos_status.pose.position.y,
        pos_status.pose.position.z, eular.z, setPos.x, setPos.y,
        setPos.z, setPos.w, eular.x, eular.y, dis_status.range);
    char* dt = packDownload(&dp);
    ROS_INFO(dt);
    serial.Write(dt, strlen(dt));
}

void modeOffBoard(ros::NodeHandle& nh)
{
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.success) {
        ROS_INFO("Offboard enabled");
    }
}

void arm(ros::NodeHandle& nh)
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
    }
}

void armAndModeOffboard(ros::NodeHandle& nh, ros::Time& last_request)
{
    // turn board into offboard mode
    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
        modeOffBoard(nh);
        last_request = ros::Time::now();
    } else {
        // check system armd
        if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            arm(nh);
            last_request = ros::Time::now();
        }
    }
}

void checkSerialCmd()
{
    char buffer[BUFFER_SIZE];
    string buff = "";
    bool save = false;
    int length = 0;

    // check serial data
    int bLength = serial.Read(buffer);

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
                serial.clear();
            }

            if (save) {
                buff.push_back(buffer[i]);
            }
        }
    }
}

void showInfo(geometry_msgs::Vector3 eular)
{
    cout << (current_state.armed == true) ? "0" : "1";
    cout << '\t' << current_state.mode << endl;
    // show imu
    cout << ros::Time::now() << '\t' << endl;

    ROS_INFO("Euler => x: [%f], y: [%f], z: [%f]", eular.x, eular.y, eular.z);

    ROS_INFO("POSE => x: [%f], y: [%f], z: [%f]", pos_status.pose.position.x,
        pos_status.pose.position.y, pos_status.pose.position.z);

    ROS_INFO("DIS => d: [%f]", dis_status.range);
}

int main(int argc, char** argv)
{
    ROS_INFO("System init ...");
    serial.Open("/dev/ttyUSB0", 115200, 8, NO, 1);
    ros::init(argc, argv, "mavpi_node");
    // ros节点
    ros::NodeHandle nh;
    //订阅器
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 10, imu_state_cb);
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "mavros/local_position/pose", 10, pose_state_cb);
    ros::Subscriber dis_sub = nh.subscribe<sensor_msgs::Range>(
        "mavros/px4flow/ground_distance", 10, dis_state_cb);
    //发布器
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "mavros/setpoint_position/local", 10);
    //
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    //连接
    // wait for FCU connection
    waitRosConnect(rate);
    //准备offboard延时
    preOffboard(rate, local_pos_pub);
    // offboard
    // modeOffBoard(nh);
    // arm
    // arm(nh);
    // log last request time
    ros::Time last_request = ros::Time::now();
    // clear serial buffer
    serial.clear();
    while (ros::ok()) {
        checkSerialCmd();

        // set status led

        // pubpos
        local_pos_pub.publish(xyz2Position(0, 0, 1.5));

        // calc eular
        geometry_msgs::Vector3 eular = quad2eular(imu_status.orientation);
        /*
           float a,b,c;
           quad(imu_status.orientation.w,
             imu_status.orientation.x,
             imu_status.orientation.y,
             imu_status.orientation.z,
             &a, &b, &c);
         */
        // show state
        showInfo(eular);

        // pub to mcu
        sendStatusUseSerial(eular);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

/**
   * @file offb_node.cpp
   * @brief offboard example node, written with mavros version 0.14.2, px4
   * flight
   * stack and tested in Gazebo SITL
   */
