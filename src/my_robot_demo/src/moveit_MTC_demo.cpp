#include <thread>
#include <chrono>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>

using Task = moveit::task_constructor::Task;
namespace stages = moveit::task_constructor::stages;
namespace solvers = moveit::task_constructor::solvers;
using CollisionObject = moveit_msgs::msg::CollisionObject;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_MTC_demo");

template <typename T>
void declareIfMissing(const rclcpp::Node::SharedPtr& node, const std::string& name, const T& value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, value);
  }
}

void configureMTCNodeParameters(const rclcpp::Node::SharedPtr& node)
{
  // Keep the demo self-contained: provide planning pipeline and IK params locally.
  declareIfMissing<std::vector<std::string>>(node, "planning_pipelines.pipeline_names", {"ompl"});
  declareIfMissing<std::string>(node, "ompl.planning_plugin", "ompl_interface/OMPLPlanner");
  declareIfMissing<std::string>(
      node,
      "ompl.request_adapters",
      "default_planner_request_adapters/AddTimeOptimalParameterization "
      "default_planner_request_adapters/FixWorkspaceBounds "
      "default_planner_request_adapters/FixStartStateBounds "
      "default_planner_request_adapters/FixStartStateCollision "
      "default_planner_request_adapters/FixStartStatePathConstraints");

  declareIfMissing<std::string>(
      node,
      "robot_description_kinematics.arm.kinematics_solver",
      "kdl_kinematics_plugin/KDLKinematicsPlugin");
  declareIfMissing<double>(node, "robot_description_kinematics.arm.kinematics_solver_search_resolution", 0.005);
  declareIfMissing<double>(node, "robot_description_kinematics.arm.kinematics_solver_timeout", 0.005);
}

CollisionObject makeCylinder(const std::string& frame_id)
{
  CollisionObject object;
  object.id = "cylinder";
  object.header.frame_id = frame_id;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = 0.18;
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = 0.03;

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = 0.45;
  pose.position.y = 0.00;
  pose.position.z = 0.09;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = CollisionObject::ADD;
  return object;
}

Task createTask(const rclcpp::Node::SharedPtr& node)
{
  Task task;
  task.stages()->setName("pick and place cylinder");
  task.loadRobotModel(node);

  auto pipeline = std::make_shared<solvers::PipelinePlanner>(node, "ompl");
  pipeline->setPlannerId("RRTConnectkConfigDefault");

  auto cartesian = std::make_shared<solvers::CartesianPath>();
  cartesian->setMaxVelocityScalingFactor(0.2);
  cartesian->setMaxAccelerationScalingFactor(0.2);
  cartesian->setStepSize(0.01);

  auto joint_interpolation = std::make_shared<solvers::JointInterpolationPlanner>();

  task.add(std::make_unique<stages::CurrentState>("current state"));

  {
    auto stage = std::make_unique<stages::MoveTo>("open gripper", joint_interpolation);
    stage->setGroup("gripper");
    stage->setGoal("gripper_open");
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::MoveTo>("move to pick", pipeline);
    stage->setGroup("arm");

    geometry_msgs::msg::PoseStamped pick_pose;
    pick_pose.header.frame_id = "base_link";
    pick_pose.pose.orientation.w = 1.0;
    pick_pose.pose.position.x = 0.45;
    pick_pose.pose.position.y = 0.0;
    pick_pose.pose.position.z = 0.24;

    stage->setGoal(pick_pose);
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::ModifyPlanningScene>("allow gripper-object collision");
    const auto* jmg = task.getRobotModel()->getJointModelGroup("gripper");
    stage->allowCollisions("cylinder", jmg->getLinkModelNamesWithCollisionGeometry(), true);
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::MoveTo>("close gripper", joint_interpolation);
    stage->setGroup("gripper");
    stage->setGoal("gripper_closed");
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
    stage->attachObject("cylinder", "tool_link");
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian);
    stage->setGroup("arm");
    stage->setIKFrame("tool_link");

    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = "base_link";
    direction.vector.z = 1.0;

    stage->setMinMaxDistance(0.08, 0.15);
    stage->setDirection(direction);
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::MoveTo>("move to place", pipeline);
    stage->setGroup("arm");

    geometry_msgs::msg::PoseStamped place_pose;
    place_pose.header.frame_id = "base_link";
    place_pose.pose.orientation.w = 1.0;
    place_pose.pose.position.x = 0.30;
    place_pose.pose.position.y = -0.35;
    place_pose.pose.position.z = 0.26;

    stage->setGoal(place_pose);
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::MoveTo>("open gripper for release", joint_interpolation);
    stage->setGroup("gripper");
    stage->setGoal("gripper_open");
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
    stage->detachObject("cylinder", "tool_link");
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid gripper-object collision");
    const auto* jmg = task.getRobotModel()->getJointModelGroup("gripper");
    stage->allowCollisions("cylinder", jmg->getLinkModelNamesWithCollisionGeometry(), false);
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::MoveRelative>("retreat", cartesian);
    stage->setGroup("arm");
    stage->setIKFrame("tool_link");

    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = "base_link";
    direction.vector.z = 1.0;

    stage->setMinMaxDistance(0.05, 0.10);
    stage->setDirection(direction);
    task.add(std::move(stage));
  }

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("moveit_MTC_demo");
  configureMTCNodeParameters(node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(makeCylinder("base_link"));
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  bool success = true;
  try {
    auto task = createTask(node);
    if (!task.plan(10)) {
      RCLCPP_ERROR(LOGGER, "MTC planning failed.");
      success = false;
    } else {
      auto solution = task.solutions().front();
      task.introspection().publishSolution(*solution);

      auto result = task.execute(*solution);
      if (result.val != result.SUCCESS) {
        RCLCPP_ERROR(LOGGER, "MTC execution failed with code %d.", result.val);
        success = false;
      } else {
        RCLCPP_INFO(LOGGER, "MTC pick and place finished.");
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(LOGGER, "Exception: %s", e.what());
    success = false;
  }

  psi.removeCollisionObjects({"cylinder"});

  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();

  return success ? 0 : 1;
}
