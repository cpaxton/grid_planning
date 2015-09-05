#include <grid/grid_planner.h>
#include <tf/transform_listener.h>

#include <exception>
#include <iostream>

#include <boost/python/tuple.hpp>

//#include <ctime>

#define _DEBUG_OUTPUT 0

/*****************************************************************************************/
#include <boost/python/stl_iterator.hpp>
using namespace boost::python;

/* Read a ROS message from a serialized string.
*/
template <typename M>
M ros_from_python(const std::string str_msg)
{
  size_t serial_size = str_msg.size();
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  for (size_t i = 0; i < serial_size; ++i)
  {
    buffer[i] = str_msg[i];
  }
  ros::serialization::IStream stream(buffer.get(), serial_size);
  M msg;
  ros::serialization::Serializer<M>::read(stream, msg);
  return msg;
}

/* Write a ROS message into a serialized string.
*/
template <typename M>
std::string ros_to_python(const M& msg)
{
  size_t serial_size = ros::serialization::serializationLength(msg);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  ros::serialization::OStream stream(buffer.get(), serial_size);
  ros::serialization::serialize(stream, msg);
  std::string str_msg;
  str_msg.reserve(serial_size);
  for (size_t i = 0; i < serial_size; ++i)
  {
    str_msg.push_back(buffer[i]);
  }
  return str_msg;
}

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
using robot_model::JointModel;
using robot_state::RobotState;
using collision_detection::CollisionRobot;
using  planning_scene_monitor::PlanningSceneMonitor;
using planning_scene_monitor::PlanningSceneMonitorPtr;

namespace grid {

  const std::string GridPlanner::TIME("time");
  const std::string GridPlanner::GRIPPER("gripper");
  const std::string GridPlanner::PS_TOPIC("monitored_planning_scene");

  /* keep robot joints up to date */
  void GridPlanner::JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    if (state) {
      state->setVariableValues(*msg); // update the current robot state
      state->update(true);
    }
    for (unsigned int i = 0; i < dof; ++i) {
      x0[i] = msg->position[i];
      x0_dot[i] = msg->velocity[i];
    }
  }

  /* get current joint positions */
  boost::python::list GridPlanner::GetJointPositions() const {
    boost::python::list res;
    for (double x: x0) {
      res.append<double>(x);
    }
    return res;
  }

  void GridPlanner::PlanningSceneCallback(const moveit_msgs::PlanningScene::ConstPtr &msg) {
    boost::mutex::scoped_lock lock(*ps_mutex);
    if (msg->is_diff) {
      scene->setPlanningSceneDiffMsg(*msg);
    } else {
      scene->setPlanningSceneMsg(*msg);
    }
  }

  GridPlanner::GridPlanner(const std::string &robot_description_,
                           const std::string &js_topic,
                           const std::string &scene_topic,
                           const double padding)

    : nh(), dof(7), num_basis(5), goal(7), x0(7), x0_dot(7), goal_threshold(7,0.1), threshold(0.1), verbose(false)
    {
      ps_mutex = std::shared_ptr<boost::mutex>(new boost::mutex);
      js_sub = nh.subscribe(js_topic.c_str(),1000,&GridPlanner::JointStateCallback,this);

      // needs to set up the Robot objects and listeners
      try {

        robot_model_loader::RobotModelLoader robot_model_loader(robot_description_);
        ROS_INFO("Loaded model from \"%s\"!",robot_description_.c_str());

        model = robot_model_loader.getModel();

      } catch (std::exception ex) {
        std::cerr << ex.what() << std::endl;
      }

      scene = std::shared_ptr<PlanningScene>(new PlanningScene(model));
      scene->getAllowedCollisionMatrix().getAllEntryNames(entry_names);
      scene->getCollisionRobotNonConst()->setPadding(padding);
      scene->propogateRobotPadding();
      state = std::shared_ptr<RobotState>(new RobotState(model));
      search_state = std::shared_ptr<RobotState>(new RobotState(model));

      //scene = PlanningScenePtr(new PlanningScene(model));
      //state = RobotStatePtr(new RobotState(model));
      //search_state = RobotStatePtr(new RobotState(model));

      boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener(ros::Duration(2.0)));
      //monitor = PlanningSceneMonitorPtr(new planning_scene_monitor::PlanningSceneMonitor(robot_description_, tf));
      //monitor->startStateMonitor(js_topic);
      //monitor->startSceneMonitor(scene_topic);

      //monitor->startPublishingPlanningScene(PlanningSceneMonitor::SceneUpdateType::UPDATE_SCENE,PS_TOPIC);
      ps_sub = nh.subscribe(scene_topic.c_str(),1000,&GridPlanner::PlanningSceneCallback,this);
    }

  /* destructor */
  GridPlanner::~GridPlanner() {
    std::cout << "Destroying planner!" << std::endl;
    js_sub.~Subscriber();
    ps_sub.~Subscriber();
    state.~shared_ptr<RobotState>();
    scene.~shared_ptr<PlanningScene>();
    search_state.~shared_ptr<RobotState>();
    std::cout << "..." << std::endl;
    model.~RobotModelPtr();
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

  void GridPlanner::PrintInfo() const {

    //moveit_msgs::PlanningScene ps_msg;
    //monitor->getPlanningScene()->getPlanningSceneMsg(ps_msg);
    //scene->setPlanningSceneMsg(ps_msg);
    scene->getCurrentStateNonConst().update(); 

    std::vector<std::string> names = scene->getWorld()->getObjectIds();

    collision_detection::CollisionRobotConstPtr robot1 = scene->getCollisionRobot();
    std::string name = robot1->getRobotModel()->getName();

    std::cout << "==========================" << std::endl;
    std::cout << "OBJECTS IN WORLD: " << std::endl;
    for (const std::string &name: names) {
      std::cout << " -- " << name << std::endl;
    }
    std::cout << "--------------------------" << std::endl;
    std:: cout << name << std::endl;
    state->printStateInfo(std::cout);

    bool colliding = scene->isStateColliding(*state,"",true);
    std::cout << "Colliding: " << colliding << std::endl;

      std::cout << "--------------------------" << std::endl;

      std::cout << "Collisions: " << std::endl;

      scene->getAllowedCollisionMatrix().print(std::cout);
    std::cout << "==========================" << std::endl;
  }

  /* try a set of motion primitives; see if they work.
   * returns an empty trajectory if no valid path was found. */
  Traj_t GridPlanner::TryPrimitives(std::vector<double> primitives) {
    boost::mutex::scoped_lock lock(*ps_mutex);

    //moveit_msgs::PlanningScene ps_msg;
    //monitor->getPlanningScene()->getPlanningSceneMsg(ps_msg);
    //scene->setPlanningSceneMsg(ps_msg);
    scene->getCurrentStateNonConst().update(); 

    Traj_t traj;
    bool colliding, bounds_satisfied;

    collision_detection::CollisionRobotConstPtr robot1 = scene->getCollisionRobot();
    std::string name = robot1->getRobotModel()->getName();

    state->update(); // not sure if this should be moved outside of the "if"

    std::vector<DMPData> dmp_list;

    unsigned int idx = 0; // where are we reading from in the primitives
    // read out the goal
    if (verbose) {
      std::cout << "Goal: ";
    }
    for (; idx < dof; ++idx) {
      goal[idx] = primitives[idx];
      if (verbose) {
        std::cout << primitives[idx] << " ";
      }
    }
    if (verbose) {
      std::cout << std::endl;
    }
    // read out the weights
    for (unsigned int i=0; i < dof; ++i) {
      dmp::DMPData dmp_;
      dmp_.k_gain = k_gain;
      dmp_.d_gain = d_gain;

      if (verbose) {
        std::cout << "Primitive " << i << ": ";
      }
      for (unsigned int j=0; j < num_basis; ++j) {
        if (verbose) {
          std::cout << primitives[idx] << " ";
        }
        dmp_.weights.push_back(primitives[idx++]);  
      }
      if (verbose) {
        std::cout << std::endl;
      }

      dmp_list.push_back(dmp_);
    }

    unsigned char at_goal;
    DMPTraj plan;
    dmp::generatePlan(dmp_list,x0,x0_dot,0,goal,goal_threshold,-1,tau,0.1,5,plan,at_goal);

    if (verbose) {
      std::cout << "--------------------------" << std::endl;

      std::cout << "at goal: " << (unsigned int)at_goal << std::endl;
      std::cout << "points: " << plan.points.size() << std::endl;
    }

    bool drop_trajectory = false;
    for (DMPPoint &pt: plan.points) {
      Traj_pt_t traj_pt;
      traj_pt.positions = pt.positions;
      traj_pt.velocities = pt.velocities;

      if (verbose) {
        std::cout << "pt: ";
        for (double &q: pt.positions) {
          std::cout << q << " ";
        }
      }

      search_state->setVariablePositions(joint_names,traj_pt.positions);
      search_state->setVariableVelocities(joint_names,traj_pt.velocities);
      search_state->update(true);

      drop_trajectory |= !scene->isStateValid(*search_state,"",verbose);

      if (verbose) {
        std::cout << " = dropped? " << drop_trajectory << std::endl;
      }


      if (drop_trajectory) {
        break;
      } else {
        traj.points.push_back(traj_pt);
      }
    }

    if (verbose) {
      std::cout << "==========================" << std::endl;
    }

    if (drop_trajectory) {
      return Traj_t(); // return an empty trajectory
    } else {
      return traj;
    }
  }

  /* try a set of motion primitives; see if they work.
   * this is aimed at the python version of the code. */
  boost::python::list GridPlanner::pyTryPrimitives(const boost::python::list &list) {
    std::vector<double> primitives = to_std_vector<double>(list);

    //moveit_msgs::PlanningScene ps_msg;
    //monitor->getPlanningScene()->getPlanningSceneMsg(ps_msg);
    //scene->setPlanningSceneMsg(ps_msg);
    scene->getCurrentStateNonConst().update(); 

#if _DEBUG_OUTPUT
    std::vector<std::string> names = scene->getWorld()->getObjectIds();

    std::cout << "==========================" << std::endl;
    std::cout << "OBJECTS IN WORLD: " << std::endl;
    for (const std::string &name: names) {
      std::cout << " -- " << name << std::endl;
    }
    std::cout << "==========================" << std::endl;
#endif

    Traj_t traj = TryPrimitives(primitives);

    boost::python::list res;

    for (const Traj_pt_t &pt: traj.points) {
      boost::python::list p;
      boost::python::list v;

      //for (const double &q: pt.positions) {
      for (unsigned int i = 0; i < pt.positions.size(); ++i) {
        p.append<double>(pt.positions[i]);
        v.append<double>(pt.velocities[i]);
      }

      res.append<boost::python::tuple>(boost::python::make_tuple(p,v));
    }

    return res;
    }

    /* update planning scene topic */
    void  GridPlanner::SetPlanningSceneTopic(const std::string &topic) {
      //monitor->startSceneMonitor(topic);
      ROS_WARN("\"GridPlanner::SetPlanningSceneTopic\" not currently implemented!");
    }

    /* configure degrees of freedom */
    void GridPlanner::SetDof(const unsigned int dof_) {
      dof = dof_;
      goal.resize(dof);
      x0.resize(dof);
      x0_dot.resize(dof);
      goal_threshold = std::vector<double>(dof,threshold);

      int i = 0;
      for (const std::string &name: search_state->getVariableNames()) {
        std::cout << "setting up joint " << i << ":" << name << std::endl;
        joint_names.push_back(name);
        i++;
        if (i >= dof) { break; }
      }
    }

    /* configure number of basis functions */
    void GridPlanner::SetNumBasisFunctions(const unsigned int num_) {
      num_basis = num_;
    }

    void GridPlanner::SetK(const double k_gain_) {
      k_gain = k_gain_;
    }

    void GridPlanner::SetD(const double d_gain_) {
      d_gain = d_gain_;
    }

    void GridPlanner::SetTau(const double tau_) {
      tau = tau_;
    }

    void GridPlanner::SetGoalThreshold(const double threshold_) {
      threshold = threshold_;
      goal_threshold = std::vector<double>(dof,threshold);
    }

    void GridPlanner::SetVerbose(const bool verbose_) {
      verbose = verbose_;
    }


    /* Are we allowed to collide? */
    void GridPlanner::SetCollisions(const std::string obj, bool allowed) {
      //for (std::string &entry: entry_names) {
      //}
      scene->getAllowedCollisionMatrixNonConst().setEntry(obj,allowed);
    }
  }

  BOOST_PYTHON_MODULE(pygrid_planner) {
    class_<grid::GridPlanner>("GridPlanner",init<std::string,std::string,std::string,double>())
      .def("Plan", &grid::GridPlanner::Plan)
      .def("AddAction", &grid::GridPlanner::AddAction)
      .def("AddObject", &grid::GridPlanner::AddObject)
      .def("TryPrimitives", &grid::GridPlanner::pyTryPrimitives)
      .def("SetK", &grid::GridPlanner::SetK)
      .def("SetD", &grid::GridPlanner::SetD)
      .def("SetTau", &grid::GridPlanner::SetTau)
      .def("SetDof", &grid::GridPlanner::SetDof)
      .def("SetNumBasisFunctions", &grid::GridPlanner::SetNumBasisFunctions)
      .def("SetGoalThreshold", &grid::GridPlanner::SetGoalThreshold)
      .def("SetVerbose", &grid::GridPlanner::SetVerbose)
      .def("PrintInfo", &grid::GridPlanner::PrintInfo)
      .def("GetJointPositions", &grid::GridPlanner::GetJointPositions)
      .def("SetCollisions", &grid::GridPlanner::SetCollisions);
  }
