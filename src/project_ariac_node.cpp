
#include "project_ariac/UR10_JointControl.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ariac_example_node");

    ros::NodeHandle node;

    UR10_JointControl ur10(node);

    ur10.gripperAction(gripper::OPEN);

    ur10.jointPosePublisher({1.85, 0.35, -0.38, 2.76, 3.67, -1.51, 0.00});

    ur10.gripperAction(gripper::CLOSE);

    ros::spin();

    return 0;
}
