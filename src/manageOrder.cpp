#include "manageOrder.hpp"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <string>

manageOrder::manageOrder(ros::NodeHandle &nh) {
    order_subscriber = nh.subscribe(
		   "/ariac/orders", 10, &manageOrder::orderCallback, this);
    called = false;
    piston = 0;
    gear = 0;
    worldFrame = "world";
    while(!called && ros::ok()) {
        ros::spinOnce();
        ros::Duration(0.02).sleep();
    }
}	

void manageOrder::orderCallback(const osrf_gear::Order::ConstPtr &order_msg) {
    ROS_INFO_STREAM(order_msg->kits.size());
    for (auto kit: order_msg->kits) {
        ROS_INFO("Working on kit type: %s", kit.kit_type.c_str());
        for (auto object: kit.objects) {
        	ROS_INFO("Working on object type: %s", object.type.c_str());
            if (std::strcmp(object.type.c_str(), "piston_rod_part") == 0) {
                piston += 1;
            } else {
                gear += 1;
            }
        }
        ROS_INFO("%d, %d", piston, gear);    
    }
    findTransform();
    called = true;
}

void manageOrder::findTransform() {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    for (int i = 0; i < piston; i++) {
        geometry_msgs::TransformStamped pose;
        srcFrame = std::string("logical_camera_1_piston_rod_part_") + std::to_string(i+1) + "_frame";
        bool flag = true;
        while (flag) {
          flag = false;  
          try {
            pose = tfBuffer.lookupTransform(srcFrame, worldFrame,
                                  ros::Time(0), ros::Duration(3));
          }
          catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
          } 
        }
        tfPose.push_back(pose);  
    }
    for (int j = 0; j < gear; j++) {
        geometry_msgs::TransformStamped pose; 
        srcFrame = std::string("logical_camera_2_gear_part_") + std::to_string(j+1) + "_frame";
        bool flag = true;
        while (flag){
          flag = false; 
          try {
            pose = tfBuffer.lookupTransform(srcFrame, worldFrame,
                                  ros::Time(0), ros::Duration(3));
          }
          catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
          } 
        }
        tfPose.push_back(pose);  
    }
}

std::vector<geometry_msgs::TransformStamped> manageOrder::getTransform() {
    return tfPose;
}