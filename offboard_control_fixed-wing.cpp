/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>
#include "tf/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>

using namespace std;
#define pi 3.141592653

mavros_msgs::State current_state;
geometry_msgs::PoseStamped local_pose;

void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose = *msg;
}
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
    bool connected = current_state.connected;
    bool armed = current_state.armed;
    std::string mode = current_state.mode;

    ROS_INFO("******** Received state cb *********");
    ROS_INFO("%sconnected to PX4", connected ? "" : "Not ");
    ROS_INFO("%sarmed", armed ? "" : "Not ");
    ROS_INFO("%s", mode.c_str());
}
float rad2ang(float input){
    return input*pi/180.0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/uav0/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav0/mavros/setpoint_position/local", 10);
    ros::Publisher local_att_pub = nh.advertise<mavros_msgs::AttitudeTarget>
        ("/uav0/mavros/setpoint_raw/attitude", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav0/mavros/set_mode");
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/uav0/mavros/local_position/pose", 10, local_pose_cb);
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    ROS_INFO("Waiting for connecting to FCU.");
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Successfully connect to FCU.");

    mavros_msgs::AttitudeTarget att_cmd;
    att_cmd.type_mask =
                            mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | //;
                            mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE;
    att_cmd.thrust = 0.8;
//    att_cmd.body_rate.x = 0;
//    att_cmd.body_rate.y = 0;
    att_cmd.body_rate.z = 0.3;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, 10.0);
    for(int i = 100; ros::ok() && i > 0; --i){
      local_att_pub.publish(att_cmd);
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("About to send thrust command.");

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    int cnt = 0,cnt_1000 = 0;

    while(ros::ok()){
        cout<<"x, y, z : "<<endl<<local_pose.pose.position.x<<endl<<local_pose.pose.position.y<<endl<<local_pose.pose.position.z<<endl;
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        //test elevator
        float expect_height = 30.0;
        float expect_height_error = expect_height - local_pose.pose.position.z;
        if(fabs(expect_height_error) >= 10.0)
            if(expect_height_error > 0) expect_height_error = 10.0;
            else expect_height_error = -10.0;
        //test code part
        //for set RPY( alieron, elevator, rudder)
        cnt++;
        if(cnt == 100) {
            cnt_1000++;
            cout<<"cnt_1000   "<<cnt_1000<<endl;
            if(cnt_1000 % 2 == 0){
                att_cmd.thrust = 0.5;
                att_cmd.body_rate.z = 0.0;
                quat.setRPY(rad2ang(-0.0), rad2ang(-expect_height_error), 0.0);
                att_cmd.orientation.x = quat.x();
                att_cmd.orientation.y = quat.y();
                att_cmd.orientation.z = quat.z();
                att_cmd.orientation.w = quat.w();
            }
            if(cnt_1000 % 2 == 1){
                att_cmd.thrust = 0.5;
                att_cmd.body_rate.z = -0.0;
                quat.setRPY(rad2ang(0.0), rad2ang(-expect_height_error), 0.0);
                att_cmd.orientation.x = quat.x();
                att_cmd.orientation.y = quat.y();
                att_cmd.orientation.z = quat.z();
                att_cmd.orientation.w = quat.w();
            }
        }
        if(cnt == 100) cnt = 0;

        local_att_pub.publish(att_cmd);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
