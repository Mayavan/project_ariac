#include "moveArm.hpp"

#include <vector>
#include <string>

moveArm::moveArm(ros::NodeHandle &nodeHandle): nh( nodeHandle ){
    joint_state_subscriber = nh.subscribe(
            "/ariac/joint_states", 10,
            &moveArm::jointStateCallback, this);
    joint_trajectory_publisher = nh.advertise<trajectory_msgs::JointTrajectory>(
            "/ariac/arm/command", 10);
    gripper = nh.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");
    gripperStateSubscriber = nh.subscribe("/ariac/gripper/state", 10, &moveArm::gripperStateCallback, this);
    called = false;
    attached = false;
    while(!called && ros::ok()) {
        //ROS_INFO("Waiting for joint feedback...");
        ros::spinOnce();
        ros::Duration(0.02).sleep();
    }
    if (!gripper.exists()) {
        gripper.waitForExistence();
    }
    attach.request.enable = 1;
    detach.request.enable = 0;
    arrivalTime = 1;
}

void moveArm::jointStateCallback(const sensor_msgs::JointState::ConstPtr &jointStateMsg)
{
    current_joint_states = *jointStateMsg;
    current_joint_states.name.pop_back();
    current_joint_states.effort.pop_back();
    current_joint_states.position.pop_back();
    current_joint_states.velocity.pop_back();
    std::vector<double> pos = {1.9993150258725745, 0.8283612304111034, -0.47862158150121203, 2.564217444355852, 3.1302356236171165, -1.5201078533494, -0.590161088164975}; 
    gear_pose.push_back(pos);
    pos = {1.7556661913415947, 0.8258945923604798, -0.4462087335570226, 2.6438218578458903, 3.393951279745265, -1.5038396922932926, -0.5269700067425571};
    gear_pose.push_back(pos);
    pos = {1.7795219161302143, 0.7201174188260127, -0.44470612613149196, 2.6770752858142997, 3.304222564211552, -1.5372031877129149, -0.44807671536402083};
    gear_pose.push_back(pos);
    pos = {1.8061755882580854, 0.6211331416982151, -0.4607750746128767, 2.713029823238684, 3.3643844310515996, -1.5103971885486511, -0.4583968345818601};
    gear_pose.push_back(pos);
    pos = {1.9743373355390013, 0.9370581153837636, -0.47860162666844364, 2.5342584661289846, 3.1575653686701366, -1.523373807376616, -0.6349526632769713};
    gear_pose.push_back(pos);
    pos = {1.9993150258725745, 0.8283612304111034, -0.47862158150121203, 2.564217444355852, 3.1302356236171165, -1.5201078533494, -0.590161088164975};
    gear_pose.push_back(pos);
    pos = {2.0249792624620397, 0.7224779178589669, -0.4856767766735901, 2.594259455759473, 3.177593495505523, -1.5101976680587295, -0.577367775981573};
    gear_pose.push_back(pos);
    pos = {2.028289114033175, 0.619018010782085, -0.45820451835509335, 2.6241797636985167, 2.997337872245463, -1.4580464307186953, -0.3020531626015206};
    gear_pose.push_back(pos);
    pos = {2.2057368210213664, 0.9286590882670054, -0.48203840098163564, 2.38431937187958, 3.0054457037437854, -1.511671305510991, -0.787629557054248};
    gear_pose.push_back(pos);
    pos = {2.2215300414306967, 0.8100089746399918, -0.46937731104484914, 2.4043896143806203, 2.9552708747228853, -1.5325001951819641, -0.7666817587477697};
    gear_pose.push_back(pos);
    pos = {2.242731858599253, 0.7006961339301091, -0.46474227435319015, 2.4290322259850248, 2.9285282708738714, -1.5327770334313446, -0.7419997707132104};
    gear_pose.push_back(pos);
    pos = {2.2613572318746655, 0.5890508919381515, -0.4573733815961658, 2.4547271087027633, 2.893788841741167, -1.5327721101537155, -0.7162361070334455};
    gear_pose.push_back(pos);
    pos = {2.381469621281054, 0.9543126294952877, -0.3984150019684094, 2.251498290137145, 2.6840963538771097, -1.5415390062874343, -0.9187928261299447};
    gear_pose.push_back(pos);
    pos = {2.402836049089771, 0.8413205270249764, -0.39203171349348853, 2.269701299052177, 2.7016679468857494, -1.5317598325187096, -0.9014087354736606};
    gear_pose.push_back(pos);
    pos = {2.4036729589603425, 0.7228897220561538, -0.3573987671615164, 2.2848690220395147, 2.569833219402487, -1.528166209658051, -0.9168282743002099};
    gear_pose.push_back(pos);
    pos = {2.428000339930982, 0.6057862974462049, -0.3559644276533884, 2.3051218499517825, 2.6111161776105996, -1.533011505684668, -0.8653628583291546};
    gear_pose.push_back(pos);
    pos = {1.746884373310083, 0.5528347886791304, -0.4412686791724223, 3.044432947795009, 3.3912761604224264, -1.5157917724921641, -0.1434396572765495};
    piston_pose.push_back(pos);
    pos = {1.746341859700843, 0.4499279733031516, -0.43302049465225334, 3.087641902710679, 3.3455451498737525, -1.5251158851790891, -0.09422732907569387};
    piston_pose.push_back(pos);
    pos = {1.7509103195415818, 0.3504865852829681, -0.43902272906888573, 3.128812020441705, 3.371784966719552, -1.5218203218243773, -0.05838259521842115};
    piston_pose.push_back(pos);
    pos = {1.747418367730159, 0.2498693109609255, -0.4344120390351849, 3.1692744436043903, 3.3558928445510143, -1.5242610727897943, -0.009329588006514733};
    piston_pose.push_back(pos);
    pos = {2.0749562046553667, 0.5963866512144856, -0.47760047277052564, 3.0895820036424233, 3.067210857660593, -1.516093403357638, -0.10276403920766697};
    piston_pose.push_back(pos);
    pos = {2.064644390097804, 0.4893180465750971, -0.4671442910932253, 3.119066164352885, 3.0615468193386906, -1.5265273182566665, -0.06261322074285935};
    piston_pose.push_back(pos);
    pos = {2.057751910916892, 0.36671534959175583, -0.4553009964552146, 3.158957782042709, 3.006506501057897, -1.5279506490455872, -0.01850950133111784};
    piston_pose.push_back(pos);
    pos = {2.0616320315647627, 0.2705747165997227, -0.4629188768425694, 3.191192732321216, 3.039907867362895, -1.5205894924807626, 0.014084181342349567};
    piston_pose.push_back(pos);
    pos = {2.3774164311264006, 0.30221974983599126, -0.391788135378512, 2.4492803959002787, 2.7130715594265173, -1.5315945288293027, -0.7245373047782984};
    piston_pose.push_back(pos);
    pos = {2.379547186249411, 0.18736556813566496, -0.3630798772943509, 2.4696582471760413, 2.6159351271190627, -1.5368937715982605, -0.6986376981537639};
    piston_pose.push_back(pos);
    pos = {2.393494686507612, 0.06926413541657013, -0.3551474345506689, 2.495003118483985, 2.6050676530682826, -1.5337058168714797, -0.666541865864386};
    piston_pose.push_back(pos);
    pos = {2.315236968260926, 0.31093071590275506, -0.4062720294586013, 3.3231094084160118, 2.7265191131365136, -1.5347592405762582, 0.18048073408653842};
    piston_pose.push_back(pos);
    called = true;
}

void moveArm::gripperStateCallback(const osrf_gear::VacuumGripperState::ConstPtr &msg) {
    currentGripperState = *msg;
    attached = msg->attached;
}

osrf_gear::VacuumGripperState moveArm::getGripperState() {
    ros::spinOnce();
    return currentGripperState;
}

bool moveArm::isGripperAttached() {
    ros::spinOnce();
    return attached;
}

bool moveArm::waitForGripperAttach(double timeout) {
    timeout = timeout <= 0? FLT_MAX:timeout;
    ros::spinOnce();
    while((!attached) && timeout > 0 && ros::ok()) {
        ROS_INFO("Retry grasp");
        release();
        ros::Duration(0.2).sleep();
        ros::spinOnce();
        grab();
        ros::Duration(0.8).sleep();
        ros::spinOnce();
        timeout -= 1.0;
    }
    return attached;
}

void moveArm::sendJointsValue(std::vector<double> joints) {
    trajectory_msgs::JointTrajectory msg;
    msg.header.stamp = ros::Time::now();
    msg.joint_names = current_joint_states.name;
    msg.points.resize(1);
    msg.points[0].positions = joints;
    msg.points[0].time_from_start = ros::Duration(arrivalTime);
    ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher.publish(msg);
    ros::Duration(arrivalTime).sleep();                         // wait for finish
    ros::spinOnce();
}

std::vector<double> moveArm::getJointsState() {
    called = false;
    while(!called && ros::ok()) {
        //ROS_INFO("Waiting for joint feedback...");
        ros::spinOnce();
        ros::Duration(0.02).sleep();
    }
    std::vector<double> joints = current_joint_states.position;
    return joints;
}

void moveArm::movetoTray(std::string type) {
    std::vector<double> my_pose;
    my_pose = {1.76,0.28,-1.38,2.76,3.27,-1.51,0.00};
    sendJointsValue(my_pose);
    recoverDrop(1, type);
        
    my_pose = {1.76,0.38,-1.38,1.5,3.27,-1.51,0.00};
    sendJointsValue(my_pose);
    recoverDrop(2, type);
        
    my_pose = {1.76,2.06,-1.38,1.5,3.27,-1.51,0.00};
    sendJointsValue(my_pose);
    recoverDrop(3, type);
        
    my_pose = {1.76,2.06,-0.63,1.5,3.27,-1.51,0.00};
    sendJointsValue(my_pose);

    ros::Duration(1.0).sleep();
    release();
    ros::Duration(1.0).sleep();
}

void moveArm::movetoBin() {
    std::vector<double> my_pose;
    my_pose = {1.76,2.06,-0.63,1.5,3.27,-1.51,0.00};
    sendJointsValue(my_pose);

    my_pose = {1.76,2.06,-1.38,1.5,3.27,-1.51,0.00};
    sendJointsValue(my_pose);

    my_pose = {1.76,0.38,-1.38,1.5,3.27,-1.51,0.00};
    sendJointsValue(my_pose);

    my_pose = {1.76,0.28,-1.38,2.76,3.27,-1.51,0.00};
    sendJointsValue(my_pose);
}

void moveArm::recoverDrop(int i, std::string type) {
    int j = i;
    std::vector<double> my_pose;
    if (!isGripperAttached()) {
       release();
       if (j == 3) {
         my_pose = {1.76,2.06,-1.38,1.5,3.27,-1.51,0.00};
         sendJointsValue(my_pose);
         j--;
       }
       if (j == 2) {
         my_pose = {1.76,0.38,-1.38,1.5,3.27,-1.51,0.00};
         sendJointsValue(my_pose);
         j--;
       }
       if (j == 1) {
         my_pose = {1.76,0.28,-1.38,1.5,3.27,-1.51,0.00};
         sendJointsValue(my_pose);
         j--;
       }
       my_pose = findNextPart(type);
       sendJointsValue(my_pose);
       grab();
       waitForGripperAttach(1.0);
       if (i == 1) {
         my_pose = {1.76,0.28,-1.38,1.5,3.27,-1.51,0.00};
         sendJointsValue(my_pose);
         i--;
       }
       if (i == 2) {
         my_pose = {1.76,0.28,-1.38,1.5,3.27,-1.51,0.00};
         sendJointsValue(my_pose);
         my_pose = {1.76,0.38,-1.38,1.5,3.27,-1.51,0.00};
         sendJointsValue(my_pose);
       }
       if (i == 3) {
         my_pose = {1.76,0.28,-1.38,1.5,3.27,-1.51,0.00};
         sendJointsValue(my_pose);
         my_pose = {1.76,0.38,-1.38,1.5,3.27,-1.51,0.00};
         sendJointsValue(my_pose);
         my_pose = {1.76,2.06,-1.38,1.5,3.27,-1.51,0.00};
         sendJointsValue(my_pose);
         i--;
       }  
    }
}

std::vector<double> moveArm::findNextPart(std::string type) {
    std::vector<double> pose;
    ROS_INFO("HERE");
    if(type.compare("piston") == 0) {
       int j = size1;
       ROS_INFO("Piston : %d", j);
       for(int i = 0; i < 7; i++) {
        pose.push_back(piston_pose[j][i]);
       }
       size1 -= 1;
       piston_pose.pop_back();
    }
    if(type.compare("gear") == 0) {
       int j = size2;
       ROS_INFO("Gear : %d", j);
       for(int i = 0; i < 7; i++) {
        pose.push_back(gear_pose[j][i]);
       }
       size2 -= 1;
       gear_pose.pop_back();
    }
    return pose;
}

void moveArm::grab() {
    //ROS_INFO("enable gripper");
    gripper.call(attach);
}

void moveArm::release() {
    //ROS_INFO("release gripper");
    gripper.call(detach);
}