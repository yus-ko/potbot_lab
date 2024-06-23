#include <potbot_lib/utility_ros.h>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/static_transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

std::string g_frame_id_odom						= "odom",
			g_frame_id_robot_base				= "base_link",
			g_dead_reckoning					= "rectangle";

bool g_publish_tf = true;

ros::Publisher g_pub_odom;
nav_msgs::Odometry g_odom;

void dead_reckoning(geometry_msgs::TwistStamped vel_msg)
{
	g_odom.header								= vel_msg.header;
	g_odom.child_frame_id						= g_frame_id_robot_base;
	g_odom.twist.twist							= vel_msg.twist;

	static double t_pre							= g_odom.header.stamp.toSec();
	double dt									= g_odom.header.stamp.toSec() - t_pre;

	double v									= g_odom.twist.twist.linear.x;
	double omega								= g_odom.twist.twist.angular.z;

	static double v_pre							= v;
	static double omega_pre						= omega;

	double theta 								= potbot_lib::utility::get_Yaw(g_odom.pose.pose.orientation);
	double x									= g_odom.pose.pose.position.x;
	double y 									= g_odom.pose.pose.position.y;
	if (g_dead_reckoning == "rectangle")
	{
		theta									+= omega*dt;
		x										+= v*cos(theta)*dt;
		y										+= v*sin(theta)*dt;
	}
	else if (g_dead_reckoning == "trapezoid")
	{
		theta									+= (omega + omega_pre)*dt/2;
		x										+= ((v + v_pre)*dt/2)*cos(theta);
		y										+= ((v + v_pre)*dt/2)*sin(theta);
	}

	g_odom.pose.pose.position.x					= x;
	g_odom.pose.pose.position.y					= y;
	g_odom.pose.pose.orientation				= potbot_lib::utility::get_Quat(0,0,theta);

	t_pre										= g_odom.header.stamp.toSec();
	v_pre										= v;
	omega_pre									= omega;

	potbot_lib::utility::print_Pose(g_odom.pose.pose);

}

void odom_broadcast(nav_msgs::Odometry odom)
{
	static tf2_ros::TransformBroadcaster dynamic_br;
	geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = odom.header.stamp;
    transformStamped.header.frame_id = odom.header.frame_id;
    transformStamped.child_frame_id = odom.child_frame_id;
    transformStamped.transform.translation.x = odom.pose.pose.position.x;
    transformStamped.transform.translation.y = odom.pose.pose.position.y;
    transformStamped.transform.translation.z = odom.pose.pose.position.z;
    transformStamped.transform.rotation.x = odom.pose.pose.orientation.x;
    transformStamped.transform.rotation.y = odom.pose.pose.orientation.y;
    transformStamped.transform.rotation.z = odom.pose.pose.orientation.z;
    transformStamped.transform.rotation.w = odom.pose.pose.orientation.w;
    dynamic_br.sendTransform(transformStamped);
}

void twist_callback(const geometry_msgs::Twist& msg)
{
	// ROS_INFO("twist callback");
	geometry_msgs::TwistStamped tm;
	tm.header.frame_id							= g_frame_id_odom;
	tm.header.stamp								= ros::Time::now();
	tm.twist									= msg;

	dead_reckoning(tm);

	if (g_publish_tf) odom_broadcast(g_odom);
	g_pub_odom.publish(g_odom);
}

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
	// ROS_INFO("pose callback");
	g_odom.pose = msg.pose;
}

void imu_callback(const sensor_msgs::Imu& msg)
{
	// ROS_INFO("imu callback");
	double roll,pitch,yaw;
	potbot_lib::utility::get_RPY(msg.orientation, roll, pitch, yaw);
	// ROS_INFO("(r,p,y) = (%f, %f, %f)", roll/M_PI*180, pitch/M_PI*180, yaw/M_PI*180);
	g_odom.pose.pose.orientation = msg.orientation;

}

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_twist_to_odometry");
	
	ros::NodeHandle n("~");
	
	double x=0,y=0,z=0,roll=0,pitch=0,yaw=0;
	n.getParam("publish_tf",					g_publish_tf);
	n.getParam("frame_id_odom",					g_frame_id_odom);
	n.getParam("frame_id_robot_base",			g_frame_id_robot_base);
	n.getParam("dead_reckoning",				g_dead_reckoning);
	n.getParam("initial_pose_x",				x);
	n.getParam("initial_pose_y",				y);
	n.getParam("initial_pose_z",				z);
	n.getParam("initial_pose_roll",				roll);
	n.getParam("initial_pose_pitch",			pitch);
	n.getParam("initial_pose_yaw",				yaw);

	ros::NodeHandle nh;
	ros::Subscriber sub_twist					= nh.subscribe("rover_odo",1,&twist_callback);
	ros::Subscriber sub_pose					= nh.subscribe("initialpose",1,&pose_callback);
	ros::Subscriber sub_imu						= nh.subscribe("imu/data",1,&imu_callback);
	g_pub_odom									= nh.advertise<nav_msgs::Odometry>("odom", 1);
	
	g_odom.pose.pose							= potbot_lib::utility::get_Pose(x,y,z,roll,pitch,yaw);

	ros::spin();
	
	return 0;
}