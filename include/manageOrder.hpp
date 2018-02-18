#include <algorithm>
#include <vector>
#include <string>
#include <cmath>
#include <ros/ros.h>
 
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_ros/transform_listener.h>

class manageOrder {
public:	
	manageOrder(ros::NodeHandle& nodeHandle);
	void orderCallback(const osrf_gear::Order::ConstPtr &order_msg);
	void findTransform();
	std::vector<geometry_msgs::TransformStamped> getTransform();

private:
	ros::Subscriber order_subscriber;
	bool called;
	int piston;
	int gear;
	std::string worldFrame;
	std::string srcFrame;
    std::vector<geometry_msgs::TransformStamped> tfPose;
};