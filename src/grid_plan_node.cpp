#include <ros/ros.h>
#include <grid/grid_planner.h>

using namespace grid;

int main(int argc, char **argv) {
  ros::init(argc,argv,"grid_plan_node");
  GridPlanner gp("robot_description","/gazebo/barrett_manager/wam/joint_states","/gazebo/planning_scene");

  std::vector<double> in;
  Traj_t res = gp.TryPrimitives(in);

  ros::Rate rate(1);
  while (ros::ok()) {
    ros::spinOnce();

    Traj_t res = gp.TryPrimitives(in);

    rate.sleep();
  }

  return 0;
}
