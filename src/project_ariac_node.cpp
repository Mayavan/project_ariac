
// #include "project_ariac/UR10_JointControl.hpp"
// #include "project_ariac/Sensor_tf.hpp"
#include "project_ariac/UR10_Control.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ariac_example_node");

    ros::NodeHandle node;

    UR10_Control ur10;

    geometry_msgs::Pose target;

    target.position.x = -0.5;
    target.position.y = -0.735;
    target.position.z = 0.724;

    // ur10.goToStart();
    ROS_INFO("Setting Target..");
    ur10.set_target(target);
    ROS_INFO("Moving");
    ur10.move();
    ROS_INFO("Picking Up");
    ur10.gripperAction(gripper::CLOSE);
    ros::Duration(1).sleep();
    ROS_INFO("Placing");
    ur10.goToStart();
    ur10.place();
    ur10.goToStart();
    // ur10.goToStart();
    return 0;
}
