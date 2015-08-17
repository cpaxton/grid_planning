#include <grid/grid_planner.h>
#include <tf/transform_listener.h>

#include <exception>
#include <iostream>

/*****************************************************************************************/
#include <boost/python/stl_iterator.hpp>
using namespace boost::python;

template< typename T >
inline
std::vector< T > to_std_vector( const boost::python::list& iterable )
{
  try {
    return std::vector< T >( boost::python::stl_input_iterator< T >( iterable ),
                             boost::python::stl_input_iterator< T >( ) );
  } catch (std::exception ex) {
    std::cerr << ex.what() << std::endl;
    return std::vector<T>();
  }
}

// Extracted from https://gist.github.com/avli/b0bf77449b090b768663.
template<class T>
struct vector_to_python
{
  static PyObject* convert(const std::vector<T>& vec)
  {
    boost::python::list* l = new boost::python::list();
    for(std::size_t i = 0; i < vec.size(); i++)
      (*l).append(vec[i]);

    return l->ptr();
  }
};

/*****************************************************************************************/

using namespace dmp;

using planning_scene::PlanningScene;
using robot_model_loader::RobotModelLoader;
using robot_model::RobotModelPtr;
using robot_state::RobotState;
using collision_detection::CollisionRobot;
using planning_scene_monitor::PlanningSceneMonitorPtr;

namespace grid {

  const std::string GridPlanner::TIME("time");
  const std::string GridPlanner::GRIPPER("gripper");

  /* keep robot joints up to date */
  void GridPlanner::JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    if (state) {
      state->setVariableValues(*msg); // update the current robot state
    }
  }

  GridPlanner::GridPlanner(const std::string &robot_description_,
                           const std::string &js_topic,
                           const std::string &scene_topic,
                           const double padding) : nh() {

    js_sub = nh.subscribe(js_topic.c_str(),1000,&GridPlanner::JointStateCallback,this);

    // needs to set up the Robot objects and listeners
    try {

      robot_model_loader::RobotModelLoader robot_model_loader(robot_description_);
      ROS_INFO("Loaded model from \"%s\"!",robot_description_.c_str());

      model = robot_model_loader.getModel();

    } catch (std::exception ex) {
      std::cerr << ex.what() << std::endl;
    }

    //scene = std::shared_ptr<PlanningScene>(new PlanningScene(model));
    //scene->getCollisionRobotNonConst()->setPadding(padding);
    //scene->propogateRobotPadding();
    state = std::shared_ptr<RobotState>(new RobotState(model));

    boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener(ros::Duration(2.0)));
    monitor = PlanningSceneMonitorPtr(new planning_scene_monitor::PlanningSceneMonitor(robot_description_, tf));
    monitor->startStateMonitor(js_topic);
    monitor->startSceneMonitor(scene_topic);
  }

  /* destructor */
  GridPlanner::~GridPlanner() {
    std::cout << "Destroying planner!" << std::endl;
    js_sub.~Subscriber();
    state.~shared_ptr<RobotState>();
    //scene.~shared_ptr<PlanningScene>();
    std::cout << "..." << std::endl;
  }

  /* add an object to the action here */
  bool GridPlanner::AddObject(const std::string &object_name) {
    ROS_WARN("\"GridPlanner::AddObject\" not yet implemented!");
    return false;
  }

  /* add an action */
  bool GridPlanner::AddAction(const std::string &action_name) {
    ROS_WARN("\"GridPlanner::AddAction\" not yet implemented!");
    return false;
  }

  /* instantiate a planning request with the given objects */
  bool GridPlanner::Plan(const std::string &action1,
                         const std::string &action2,
                         const std::unordered_map<std::string, std::string> &object_mapping)
  {
    return false;
  }


  /* try a set of motion primitives; see if they work.
   * returns an empty trajectory if no valid path was found. */
  Traj_t GridPlanner::TryPrimitives(std::vector<double> primitives) {

    Traj_t traj;

    collision_detection::CollisionRobotConstPtr robot1 = monitor->getPlanningScene()->getCollisionRobot();
    std::string name = robot1->getRobotModel()->getName();
    std:: cout << name << std::endl;
    state->update(true);
    state->printStateInfo(std::cout);

    bool colliding = monitor->getPlanningScene()->isStateColliding(*state,"",true);
    std::cout << "Colliding: " << colliding << std::endl;

    return traj;
  }

  /* try a set of motion primitives; see if they work.
   * this is aimed at the python version of the code. */
  boost::python::list GridPlanner::pyTryPrimitives(const boost::python::list &list) {
    std::vector<double> primitives = to_std_vector<double>(list);

    Traj_t traj = TryPrimitives(primitives);

    boost::python::list res;
    return res;
  }

  /* update planning scene topic */
  void  GridPlanner::SetPlanningSceneTopic(const std::string &topic) {
    monitor->startSceneMonitor(topic);
  }

}

BOOST_PYTHON_MODULE(pygrid_planner) {
  class_<grid::GridPlanner>("GridPlanner",init<std::string,std::string,std::string,double>())
    .def("Plan", &grid::GridPlanner::Plan)
    .def("AddAction", &grid::GridPlanner::AddAction)
    .def("AddObject", &grid::GridPlanner::AddObject)
    .def("TryPrimitives", &grid::GridPlanner::pyTryPrimitives)
    ;
}


