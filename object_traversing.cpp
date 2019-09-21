/**
 * @file aero_offboard_ctrl_node.cpp
 * @brief Aero Offboard Control example node, written with mavros version 0.21.2, px4 flight
 * stack and tested in Gazebo SITL & jMAVSIM.
 * Original source code: MAVROS OFFBOARD example from: https://dev.px4.io/en/ros/mavros_offboard.html
 *
 * This example is summariesed below:
 * 1. Listens for state of the Aero Flight Controller.
 * 2. Set offboard setpoint location before changing OFFBOARD (Otherwise mode switch will be rejected)
 * 3. Switches to OFFBOARD mode.
 * 4. When Aero Flight Controller goes to OFFBOARD mode, we continue publishing setpoint position with altitude of 2
 * meters.
 * 5. We keep receiving Aero FCU state callabck.
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>
#include <fstream>
#include <random>

using namespace std;
using namespace ros;

// Callback that gets called at a fixed rate reporting FCU state: connection state, armed, mode, etc.
void printStateCB(const mavros_msgs::State::ConstPtr& msg, mavros_msgs::State* state)
{
  *state = *msg;

  bool connected = state->connected;
  bool armed = state->armed;
  std::string mode = state->mode;

  ROS_INFO("******** Received state cb *********");
  ROS_INFO("%sconnected to PX4", connected ? "" : "Not ");
  ROS_INFO("%sarmed", armed ? "" : "Not ");
  ROS_INFO("%s", mode.c_str());
  ROS_INFO("------------------------------------");
}

void current_positionCB(const geometry_msgs::PoseStamped::ConstPtr& msg, geometry_msgs::PoseStamped* PositionTarget)
{
  *PositionTarget = *msg;
    //just update do nothing
}

struct point3d
{
    float x;
    float y;
    float z;
};

float norm2(point3d error){
    return sqrt(error.x * error.x + error.y * error.y);
}
point3d path_planning(point3d destinaion, point3d cur_pos, float alpha){
    point3d error,output;
    error.x = destinaion.x - cur_pos.x;
    error.y = (destinaion.y - cur_pos.y)*(1.0 - alpha);
    error.z = destinaion.z - cur_pos.z;
    float t_step = norm2(error);
    float step = 0.3;
    if(t_step > step){
        output.x = cur_pos.x + step * error.x / t_step;
        output.y = cur_pos.y + step * error.y / t_step;
        output.z = cur_pos.z + step * error.z;
    }else{
        output.x = cur_pos.x + error.x, output.y = cur_pos.y + error.y , output.z = cur_pos.z + error.z;
    }
    return output;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  mavros_msgs::State current_state;

  ros::Subscriber state_sub =
      nh.subscribe<mavros_msgs::State>("/uav1/mavros/state", 10, boost::bind(printStateCB, _1, &current_state));
  geometry_msgs::PoseStamped current_position;
  ros::Subscriber current_position_sub =
          nh.subscribe<geometry_msgs::PoseStamped>("/uav1/mavros/local_position/pose", 1000 , boost::bind(current_positionCB, _1, &current_position));
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav1/mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav1/mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav1/mavros/set_mode");

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  ofstream outFile;
  outFile.open("/home/nuc-fixedwing/catkin_ws/src/aero_offboard_ctrl/record/arc1.txt");
  // wait for FCU connection
  while (ros::ok() && current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 4;

  // send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();
  const double mean = 0.0;//均值
  const double stddev = 3;//标准差
  std::default_random_engine generator;
  std::normal_distribution<double> dist(mean, stddev);
  while (ros::ok())
  {
    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
      if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
      {
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    }
    else
    {
      if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
      {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }


    point3d destination;
    point3d cur_pos, output;
    destination.x=10 + dist(generator)/8;
    destination.y=10 + dist(generator);
    destination.z=4 + + dist(generator)/8;
    cur_pos.x = current_position.pose.position.x;
    cur_pos.y = current_position.pose.position.y;
    cur_pos.z = current_position.pose.position.z;
    float alpha = 0.5;
    output = path_planning(destination,cur_pos, alpha);
    pose.pose.position.x = output.x;
    pose.pose.position.y = output.y;
    pose.pose.position.z = output.z;
    cout<<"dest = "<<destination.x<< " "<<destination.y <<" "<<destination.z<<endl;
    cout<<"cur_pose = "<<cur_pos.x<< " "<<cur_pos.y <<" "<<cur_pos.z<<endl;
    cout<<"output = "<<output.x<< " "<<output.y <<" "<<output.z<<endl;
    outFile <<cur_pos.x<< " "<<cur_pos.y <<" "<<cur_pos.z<<endl;
    local_pos_pub.publish(pose);

    ros::spinOnce();
    rate.sleep();
  }
  outFile.close();
  return 0;
}
