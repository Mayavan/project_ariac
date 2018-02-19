
// #include "project_ariac/UR10_JointControl.hpp"
// #include "project_ariac/Sensor_tf.hpp"
#include "project_ariac/UR10_Control.hpp"
// #include "project_ariac/Manger.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "ariac_example_node");

  ros::NodeHandle node;

  UR10_Control ur10;

  geometry_msgs::Pose target;

  target.position.x = -0.5;
  target.position.y = -0.735;
  target.position.z = 0.724;

  ur10.pickAndPlace(target);

  // Manger mangement;

  // while(!mangement.isRaedy()) {
  //     ros::spinOnce();
  //     ros::Duration(1.0).sleep();
  // }
  // start_competition(node);

  return 0;
}
