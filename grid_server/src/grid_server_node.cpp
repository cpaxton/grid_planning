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
#include <grid_plan_msgs/ConfigureObjects.h>

#include <grid/server_params.h>

using namespace grid;

Params params;
ServerParams server_params;

GridPlanner *gp;
RobotKinematicsPtr robot;

bool handleTaskPlanRequest(grid_plan_msgs::TaskPlanRequestRequest &req,
                           grid_plan_msgs::TaskPlanRequestResponse &res)
{

  return true;
}

bool handleConfigureObjects(grid_plan_msgs::ConfigureObjectsRequest &req,
                            grid_plan_msgs::ConfigureObjectsResponse &res)
{
  return true;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "grid_plan_server");

  ros::NodeHandle nh;

  params = readRosParams();
  readServerParams(server_params);
  RobotKinematicsPtr robot = RobotKinematicsPtr(new RobotKinematics("robot_description","wam/base_link","wam/wrist_palm_link"));
  GridPlanner gp("robot_description","/gazebo/barrett_manager/wam/joint_states","/gazebo/raw_planning_scene",0.01);

  /***************************************************************************/
  ros::ServiceServer taskPlanService = nh.advertiseService(
      "plan_task",
      handleTaskPlanRequest);
  ros::ServiceServer configureObjectsService = nh.advertiseService(
      "configure_objects",
      handleConfigureObjects);

  ros::spin();

  return 0;
}
