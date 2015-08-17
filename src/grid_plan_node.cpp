#include <ros/ros.h>
#include <grid/grid_planner.h>

using namespace grid;

int main(int argc, char **argv) {
  ros::init(argc,argv,"grid_plan_node");
  GridPlanner gp("robot_description","/gazebo/barrett_manager/wam/joint_states","/gazebo/planning_scene");
  gp.SetDof(7);
  gp.SetNumBasisFunctions(5);

  std::vector<double> in;

  double primitives[] = {
    // goal state
    1.4051760093677244, -1.1104719560033205, -1.9033419999549013, 1.8269932570905123, 1.2499923807229427, 0.08964526401203354, -0.8798027314692156,
    // parameters for the dmp
    0.1,0.1,0.1,0.1,0.1,
    0.1,0.1,0.1,0.1,0.1,
    0.1,0.1,0.1,0.1,0.1,
    0.1,0.1,0.1,0.1,0.1,
    0.1,0.1,0.1,0.1,0.1,
    0.1,0.1,0.1,0.1,0.1,
    0.1,0.1,0.1,0.1,0.1
  };

  in = std::vector<double>(&primitives[0],&primitives[7+35+1]);
  Traj_t res = gp.TryPrimitives(in);

  ros::Rate rate(1);
  while (ros::ok()) {
    ros::spinOnce();

    Traj_t res = gp.TryPrimitives(in);

    rate.sleep();
  }

  return 0;
}
