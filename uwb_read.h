#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <math.h>
#include <std_msgs/String.h>
#include </home/mr-zhang/zhang/catkin_ws/devel/include/mavros_msgs/uwb_read_ros.h>

using namespace std;
using namespace boost::asio;

namespace uwb_comm
{
class uwb_read
{
public:
	uwb_read();
	~uwb_read();
//private:
	uint8_t buf[128];
	//定义该标签与各个基站的距离 m
	double dis0;
	double dis1;
	double dis2;
	double dis3;
	double dis4;
	double dis5;
	double dis6;
	double dis7;
	
	//定义该标签的位置 m
	double tag_position_x;
	double tag_position_y;
	double tag_position_z;
	
	//定义该标签的速度 m/s
	double tag_velocity_x;
	double tag_velocity_y;
	double tag_velocity_z;
	
	//定义三轴陀螺仪的角速度 rad/s
	double gyro_x;
	double gyro_y;
	double gyro_z;
	
	//定义三轴加速度 m/s^2
	double acc_x;
	double acc_y;
	double acc_z;
	
//private:
	//转换数据 输入：首字符地址，转换位数 输出：10进制数据
	double trans_data(uint8_t *buf,int b_start,int b_n);
	
//private:
	ros::NodeHandle n;
	ros::Publisher uwb_data_;
	double main_loop_duration_;
	ros::Timer main_loop_timer_;
	void mainLoop(const ros::TimerEvent& event);
	mavros_msgs::uwb_read_ros uwb_data_ros;

        serial_port sp;//定义传输的串口
};
}
