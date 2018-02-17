#include <algorithm>
#include <vector>
#include <cmath>
#include <ros/ros.h>
 
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <moveArm.hpp>

/// Start the competition by waiting for and then calling the start ROS Service.
void start_competition(ros::NodeHandle & node) {
   // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
   ros::ServiceClient start_client =
     node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
   // If it's not already ready, wait for it to be ready.
   // Calling the Service using the client before the server is ready would fail.
   if (!start_client.exists()) {
     ROS_INFO("Waiting for the competition to be ready...");
     start_client.waitForExistence();
     ROS_INFO("Competition is now ready.");
   }
   ROS_INFO("Requesting competition start...");
   std_srvs::Trigger srv;  // Combination of the "request" and the "response".
   start_client.call(srv);  // Call the start Service.
   if (!srv.response.success) {  // If not successful, print out why.
     ROS_ERROR_STREAM("Failed to start the competition" << srv.response.message);
   }
 }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ariac_qual1");
    ros::NodeHandle nh;                                 // standard ros node handle
    moveArm robotArm(nh);
    // OrderManager comp(nh);
    std::vector<double> my_pose = robotArm.getJointsState();
    //ROS_INFO("Current Joint state pose: " << my_pose);
    start_competition(nh);
    ROS_INFO("Competition started!");

    ros::Duration(5).sleep();
    bool attached = false;
    while (ros::ok()) {
        my_pose[0] = 1.85;
        my_pose[1] = 0.35;
        my_pose[2] = -0.38;
        my_pose[3] = 2.76;
        my_pose[4] = 3.67;
        my_pose[5] = -1.51;
        my_pose[6] = 0.00;
        robotArm.sendJointsValue(my_pose);

        robotArm.grab();
        //while (!attached) {
        robotArm.waitForGripperAttach(1.0);
        //}

        my_pose[0] = 1.76;
        my_pose[1] = 0.28;
        my_pose[2] = -1.38;
        my_pose[3] = 2.76;
        my_pose[4] = 3.27;
        my_pose[5] = -1.51;
        my_pose[6] = 0.00;
        robotArm.sendJointsValue(my_pose);
        
        my_pose[0] = 1.76;
        my_pose[1] = 0.38;
        my_pose[2] = -1.38;
        my_pose[3] = 1.5;
        my_pose[4] = 3.27;
        my_pose[5] = -1.51;
        my_pose[6] = 0.0;
        robotArm.sendJointsValue(my_pose);
        
        my_pose[0] = 1.76;
        my_pose[1] = 2.06;
        my_pose[2] = -1.38;
        my_pose[3] = 1.5;
        my_pose[4] = 3.27;
        my_pose[5] = -1.51;
        my_pose[6] = 0.0;
        robotArm.sendJointsValue(my_pose);
        
        my_pose[0] = 1.76;
        my_pose[1] = 2.06;
        my_pose[2] = -0.63;
        my_pose[3] = 1.5;
        my_pose[4] = 3.27;
        my_pose[5] = -1.51;
        my_pose[6] = 0.0;
        robotArm.sendJointsValue(my_pose);
        ros::Duration(1.0).sleep();
        robotArm.release();
        ros::Duration(1.0).sleep();

        my_pose[0] = 1.76;
        my_pose[1] = 2.06;
        my_pose[2] = -0.63;
        my_pose[3] = 1.5;
        my_pose[4] = 3.27;
        my_pose[5] = -1.51;
        my_pose[6] = 0.0;
        robotArm.sendJointsValue(my_pose);

        my_pose[0] = 1.76;
        my_pose[1] = 2.06;
        my_pose[2] = -1.38;
        my_pose[3] = 1.5;
        my_pose[4] = 3.27;
        my_pose[5] = -1.51;
        my_pose[6] = 0.0;
        robotArm.sendJointsValue(my_pose);

        my_pose[0] = 1.76;
        my_pose[1] = 2.06;
        my_pose[2] = -1.38;
        my_pose[3] = 1.5;
        my_pose[4] = 3.27;
        my_pose[5] = -1.51;
        my_pose[6] = 0.0;
        robotArm.sendJointsValue(my_pose);

        my_pose[0] = 1.76;
        my_pose[1] = 0.28;
        my_pose[2] = -1.38;
        my_pose[3] = 2.76;
        my_pose[4] = 3.27;
        my_pose[5] = -1.51;
        my_pose[6] = 0.00;
        robotArm.sendJointsValue(my_pose);

        my_pose[0] = 1.85;
        my_pose[1] = 0.35;
        my_pose[2] = -0.38;
        my_pose[3] = 2.76;
        my_pose[4] = 3.67;
        my_pose[5] = -1.51;
        my_pose[6] = 0.00;
        robotArm.sendJointsValue(my_pose);
        ros::Duration(21.0).sleep();
    }
    return 0;
}
