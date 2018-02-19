#include <algorithm>
#include <vector>
#include <cmath>
#include <ros/ros.h>
 
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/AGVControl.h>
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
#include <manageOrder.hpp>

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

void send_order(ros::NodeHandle & node) {
   // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
   ros::ServiceClient agv_client =
     node.serviceClient<osrf_gear::AGVControl>("/ariac/agv1");
   // If it's not already ready, wait for it to be ready.
   // Calling the Service using the client before the server is ready would fail.
   if (!agv_client.exists()) {
     ROS_INFO("Waiting for the AGV client to be ready...");
     agv_client.waitForExistence();
     ROS_INFO("AGV client is now ready.");
   }
   ROS_INFO("Requesting AGV to complete order...");
   osrf_gear::AGVControl srv;  // Combination of the "request" and the "response".
   srv.request.kit_type = 'order_0_kit_0'; 
   agv_client.call(srv);  // Call the start Service.
   if (!srv.response.success) {  // If not successful, print out why.
     ROS_ERROR_STREAM("Failed to start the competition");
   }
 }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "project_ariac_node_1b");
    ros::NodeHandle nh;                                 // standard ros node handle
    moveArm robotArm(nh);
    std::vector<double> my_pose = robotArm.getJointsState();
    start_competition(nh);
    ROS_INFO("Competition started!");
    ros::Duration(1).sleep();
    
    // pick the 1st part
    my_pose = robotArm.findNextPart("piston");
    robotArm.sendJointsValue(my_pose);
    robotArm.grab();
    robotArm.waitForGripperAttach(1.0);

    // move to the tray and drop
    robotArm.movetoTray("piston");
         
    // move back to the bin and pick 2nd part
    robotArm.movetoBin();
    my_pose = robotArm.findNextPart("piston");
    robotArm.sendJointsValue(my_pose);
    robotArm.grab();
    robotArm.waitForGripperAttach(1.0);

    // move to the tray and drop
    robotArm.movetoTray("piston");

    // move back to the bin and pick 3rd part
    robotArm.movetoBin();
    my_pose = robotArm.findNextPart("gear");
    robotArm.sendJointsValue(my_pose);
    robotArm.grab();
    robotArm.waitForGripperAttach(1.0);

    // move to the tray and drop
    robotArm.movetoTray("gear");

    // move back to the bin and pick 4th part
    robotArm.movetoBin();
    my_pose = robotArm.findNextPart("gear");
    robotArm.sendJointsValue(my_pose);
    robotArm.grab();
    robotArm.waitForGripperAttach(1.0);
        
    // move to the tray and drop
    robotArm.movetoTray("gear");

    // move back to the bin and pick 5th part
    robotArm.movetoBin();
    my_pose = robotArm.findNextPart("gear");
    robotArm.sendJointsValue(my_pose);
    robotArm.grab();
    robotArm.waitForGripperAttach(1.0);
     
    // move to the tray and drop
    robotArm.movetoTray("gear");
    
    // send the agv1 to complete order
    send_order(nh);

    // move the arm back to its original position
    robotArm.movetoBin();
    my_pose = {1.512,0.000,-1.12,3.138,3.767,-1.506,0.000};
    robotArm.sendJointsValue(my_pose);
    ros::Duration(1.0).sleep();

    return 0;
}
