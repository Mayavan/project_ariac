/**
 * @file UR10_Control.cpp
 * @author     Ravi Bhadeshiya
 * @version    0.1
 * @brief      Class for controlling ur10 arm
 *
 * @copyright  MIT License (c) 2017 Ravi Bhadeshiya
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "project_ariac/UR10_Control.hpp"

UR10_Control::UR10_Control():ur10_("manipulator"){

    // ur10_.setPlanningTime(5);
    ur10_.setPlanningTime(50);
    ur10_.setNumPlanningAttempts(5);
    ur10_.setPlannerId("RRTConnectkConfigDefault");
}

UR10_Control::~UR10_Control(){

}

void UR10_Control::set_target(const geometry_msgs::Pose& target_){
        ur10_.setPoseTarget(target_);
}

bool UR10_Control::plan() {
    // bool success = ur10_.plan(planner);
    ROS_INFO("Planning start!");
    bool success = (ur10_.plan(planner) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success) ROS_INFO("Planned success.");
    else ROS_INFO("Planned unsucess!");
    return success;
}

void UR10_Control::move() {
    if(this->plan()) {
        // ur10_.move();
        ur10_.execute(planner);
        // ur10_.asyncExecute(planner);
        ros::Duration(1.0).sleep();
    }
}
