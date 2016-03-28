#include <ros/ros.h>

#include <grid/skill.h>
#include <grid/task_model.h>
#include <grid/robot_kinematics.h>
#include <grid/grid_planner.h>
#include <grid/visualize.h>
#include <grid/utils/params.h>
#include <grid/wam/input.h>

#include <grid_plan_msgs/ActionConfig.h>
#include <grid_plan_msgs/ObjectConfig.h>
#include <grid_plan_msgs/TaskPlanRequest.h>

using namespace grid;

bool handleActionSequence(grid_plan_msgs::TaskPlanRequestRequest &req,
                          grid_plan_msgs::TaskPlanRequestResponse &res)
{


}

int main(int argc, char **argv) {

  ros::init(argc, argv, "grid_plan_server");


  return 0;
}
