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
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>

class moveArm {
public:
    moveArm(ros::NodeHandle& nodeHandle);

    void sendJointsValue(std::vector<double> joints);
    std::vector<double> getJointsState();
    void grab();
    void release();
    osrf_gear::VacuumGripperState getGripperState();
    bool isGripperAttached();
    bool waitForGripperAttach(double timeout);

private:
    ros::NodeHandle nh;

    ros::Publisher joint_trajectory_publisher;
    ros::Subscriber joint_state_subscriber;
    ros::ServiceClient gripper;
    ros::Subscriber gripperStateSubscriber;

    sensor_msgs::JointState current_joint_states;
    osrf_gear::VacuumGripperState currentGripperState;
    bool called;
    bool attached;
    osrf_gear::VacuumGripperControl attach;
    osrf_gear::VacuumGripperControl detach;
    double arrivalTime;

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_state_msg);
    void gripperStateCallback(const osrf_gear::VacuumGripperState::ConstPtr &msg);
};
