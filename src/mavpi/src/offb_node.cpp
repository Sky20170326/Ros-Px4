/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
//#include <boost/asio.hpp>

#define simulator

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
}

geometry_msgs::PoseStamped xyz2Position (float x,float y,float z)
{
        geometry_msgs::PoseStamped pose;

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;

        return pose;
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "offb_node");
        ros::NodeHandle nh;

        ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                            ("mavros/state", 10, state_cb);

        ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                               ("mavros/setpoint_position/local", 10);

        //ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
        //                                       ("mavros/setpoint_raw/local",10);

        ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                                   ("mavros/cmd/arming");

        ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                                     ("mavros/set_mode");

        //the setpoint publishing rate MUST be faster than 2Hz
        ros::Rate rate(20.0);

        // wait for FCU connection
        while(ros::ok() && current_state.connected) {
                ros::spinOnce();
                rate.sleep();
        }

        //send a few setpoints before starting
        for(int i = 100; ros::ok() && i > 0; --i) {
                local_pos_pub.publish(xyz2Position(0,0,1.5));
                ros::spinOnce();
                rate.sleep();
        }

        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        ros::Time last_request = ros::Time::now();

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

                //set status led

                //check timeout

                //pubpos
                local_pos_pub.publish(xyz2Position (0,0,1.5));
                //pubyaw


                ros::spinOnce();
                rate.sleep();
        }

        return 0;
} /**
   * @file offb_node.cpp
   * @brief offboard example node, written with mavros version 0.14.2, px4 flight
   * stack and tested in Gazebo SITL
   */
