
// #include "project_ariac/UR10_JointControl.hpp"
// #include "project_ariac/Sensor_tf.hpp"
#include "project_ariac/UR10_Control.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ariac_example_node");

    ros::NodeHandle node;

    UR10_Control ur10;

    geometry_msgs::Pose target;

    target.orientation.x = 0;
    target.orientation.y = 0;
    target.orientation.z = 0;
    target.orientation.w = 0;

    // we set positions for our pose
    target.position.x = -0.5;
    target.position.y = -0.3;
    target.position.z = 0.7;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("Setting Target..");
    ur10.set_target(target);
    ROS_INFO("Moving");
    ur10.move();
    // UR10_JointControl ur10(node);

    // ur10.gripperAction(gripper::OPEN);

    // ur10.jointPosePublisher({1.85, 0.35, -0.38, 2.76, 3.67, -1.51, 0.00});

    // ur10.gripperAction(gripper::CLOSE);

    // ros::spin();

    return 0;
}
