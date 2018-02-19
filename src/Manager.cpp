#include "project_ariac/Manager.cpp"

Manager::Manager() {
  ros::Subscriber logical_camera_1 = nh.subscribe(
      "/ariac/logical_camera_1", 10, &Manager::logical_camera_callback_1, this);
  ros::Subscriber logical_camera_2 = nh.subscribe(
      "/ariac/logical_camera_2", 10, &Manager::logical_camera_callback_2, this);
  ros::Subscriber orders_subscriber =
      nh.subscribe("/ariac/orders", 10, &Manager::order_callback, this);
}

Manager::~Manager() {
    for(auto i:order) {
        ROS_INFO_STREAM(i.first);
        for(auto j:i.second){
            ROS_INFO_STREAM(j);
        }
    }
}

void Manager::logical_camera_callback_1(
    const osrf_gear::LogicalCameraImage::ConstPtr& image_msg) {
  if (l1_flag) return;
  size_t count = 1;
  for (const auto& itr : image_msg->models) {
    std::string partFrame =
        "logical_camera_1_" + itr.type + "_" + count + "_frame";
    inventory[itr.type].insert(partFrame);
    count++;
  }
  l1_flag = true;
}

void Manager::logical_camera_callback_2(
    const osrf_gear::LogicalCameraImage::ConstPtr& image_msg) {
  if (l2_flag) return;
  size_t count = 1;
  for (const auto& itr : image_msg->models) {
    std::string partFrame =
        "logical_camera_2_" + itr.type + "_" + count + "_frame";
    inventory[itr.type].insert(partFrame);
    count++;
  }
  l2_flag = true;
}

void Manager::order_callback(const osrf_gear::Order::ConstPtr& order_msg) {
  for (const auto& itr : order_msg->order_id[0].kits[0].objects) {
    order[itr.type].push_back(inventory[itr.type].front());
    inventory[itr.type].pop_front()
  }
}

void Manager::isReady() {
    return l1_flag && l2_flag;
}
