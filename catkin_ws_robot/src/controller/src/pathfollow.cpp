#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
double ang=0;
void NewDir(const nav_msgs::Odometry::ConstPtr& msg){
	ang=msg->pose.pose.orientation.z;
	double angX=msg->pose.pose.orientation.x;
	double angY=msg->pose.pose.orientation.y;
	ROS_INFO("Ang: %f, X: %f, Y: %f",ang,angX,angY);

}



int main(int argc, char** argv){
	ros::init(argc,argv,"pathfollow");
	ros::NodeHandle nh;
	ros::Subscriber sub=nh.subscribe("odom",1,NewDir);
	ros::spin();
	return 0;
}
