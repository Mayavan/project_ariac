
#include "project_ariac/Sensor_tf.hpp"

Sensor_tf::Sensor_tf() {}

Sensor_tf::Sensor_tf(ros::NodeHandle nh_) {
  nh = nh_;
  logical_camera_subscriber =
      nh.subscribe(topic, 10, &Sensor_tf::logical_camera_callback, this);
  parts_pose_publisher =
      nh.advertise<geometry_msgs::PoseArray>(topic + "_PoseArray", 10);
}

Sensor_tf::Sensor_tf(ros::NodeHandle nh_, const std::string &topic_) {
  nh = nh_;
  topic = topic_;
  ros::Subscriber logical_camera_subscriber =
      nh.subscribe(topic, 10, &Sensor_tf::logical_camera_callback, this);
  parts_pose_publisher =
      nh.advertise<geometry_msgs::PoseArray>(topic + "_PoseArray", 10);
}

Sensor_tf::~Sensor_tf() {}

void Sensor_tf::logical_camera_callback(
    const osrf_gear::LogicalCameraImage::ConstPtr &image_msg) {
  geometry_msgs::PoseArray msg;
  msg.poses.resize(image_msg->models.size());
  for (size_t i = 0; i < image_msg->models.size(); i++) {
    // for(const auto &model : image_msg->models) {
    geometry_msgs::PoseStamped inPose, outPose;

    std::string str1 = topic + "_" + image_msg->models[i].type + "_" +
                std::to_string(i) + "_frame";

    std::string s = "/ariac/";std::string::size_type idx = str1.find(s);
    if (idx != std::string::npos) str1.erase(idx, s.length());

    inPose.header.frame_id = str1;

    // inPose.pose = model.pose;
    inPose.pose = image_msg->models[i].pose;

    bool tferr = true;
    ROS_INFO_STREAM(inPose.header.frame_id);
    while (tferr && ros::ok()) {
      try {
        tf::StampedTransform transform;
        inPose.header.stamp = ros::Time(0);
        // tf_listener.transformPose("world", inPose, outPose);
        tf_listener.lookupTransform(inPose.header.frame_id,"world",ros::Time(0),transform);
        tferr = false;
      } catch (tf::TransformException &exception) {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
      }
    }
    // msg.poses[i] = outPose.pose;
  }
  ROS_INFO_STREAM("Populated Pose msg:\n" << msg);
  // parts_pose_publisher.publish(msg);
}
