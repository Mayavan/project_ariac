
/**
 * @file project_ariac_node.cpp
 * @author     Ravi Bhadeshiya
 * @version    2.0
 * @brief      The main node for project_ariac
 *
 * @copyright  BSD 3-Clause License (c) 2018 Ravi Bhadeshiya
 **/

#include "project_ariac/Manager.hpp"
#include "project_ariac/UR10_Control.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "project_ariac_node");

  ros::NodeHandle node;
  ros::NodeHandle private_node_handle("~");

  bool run;
  private_node_handle.param("run", run, false);

  UR10_Control ur10(private_node_handle);

  if (run) {
    ur10.goToStart();
    return 0;
  }

  Manager m(node);
  m.start_competition();
  m.checkInventory();
  m.finishOrder();
  m.send_order();
  m.end_competition();

  return 0;
}
