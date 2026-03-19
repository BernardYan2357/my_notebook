#include <thread>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using PlanningSceneInterface = moveit::planning_interface::PlanningSceneInterface;

int main(int argc, char *argv[])
{
    // 初始化 MoveIt2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("moveit_PSI_demo");
    rclcpp::executors::SingleThreadedExecutor executor; // 创建单线程执行器
    executor.add_node(node); // 将节点添加到执行器中
    auto spinner = std::thread([&executor](){executor.spin();}); // 启动执行器的线程
    auto arm = MoveGroupInterface(node, "arm");
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);
    arm.setPlanningTime(5.0); // 设置规划时间
    arm.setNumPlanningAttempts(10); // 设置尝试次数

    // 使用当前末端位姿作为基准，保证目标姿态可行性更高
    auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.position.x = 0.00;
    msg.position.y = 1.00;
    msg.position.z = 0.60;
    return msg;
    }();

    // 创建一个碰撞对象，并将其添加到规划场景中
    auto const collision_object = [frame_id = arm.getPlanningFrame()] { //定义一个lambda函数
      moveit_msgs::msg::CollisionObject collision_object;
      collision_object.header.frame_id = frame_id.empty() ? "base_link" : frame_id;
      collision_object.id = "box";
      // 定义一个 SolidPrimitive 对象，表示一个简单的几何形状，用于描述碰撞对象的形状
      shape_msgs::msg::SolidPrimitive primitive;
      // 定义盒子的尺寸（以米为单位）
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3); // 盒子有三个维度：长度、宽度和高度
      primitive.dimensions[primitive.BOX_X] = 0.30;
      primitive.dimensions[primitive.BOX_Y] = 0.10;
      primitive.dimensions[primitive.BOX_Z] = 0.30;
      // 定义盒子的位置（相对于 frame_id）
      geometry_msgs::msg::Pose box_pose;
      box_pose.position.x = 0.00;
      box_pose.position.y = 0.50;
      box_pose.position.z = 0.60;
      collision_object.primitives.push_back(primitive); // 将定义的几何形状添加到碰撞对象中
      collision_object.primitive_poses.push_back(box_pose); // 将定义的位置添加到碰撞对象中
      collision_object.operation = collision_object.ADD; // 设置操作类型为添加
      return collision_object;
    }();

    // 目标规划->执行
    RCLCPP_INFO(node->get_logger(),
      "Target pose (x=%.3f, y=%.3f, z=%.3f)",
      target_pose.position.x,target_pose.position.y,target_pose.position.z);
    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose);
    MoveGroupInterface::Plan plan_without_obstacle;
    auto const ok_without_obstacle = static_cast<bool>(arm.plan(plan_without_obstacle));
    if (ok_without_obstacle) {
      RCLCPP_INFO(node->get_logger(), "Plan succeeded. Executing trajectory.");
      auto const exec_ok = static_cast<bool>(arm.execute(plan_without_obstacle));
      RCLCPP_INFO(node->get_logger(), "Execute result: %s", exec_ok ? "success" : "failed");
    } else {
      RCLCPP_WARN(node->get_logger(), "Plan failed. Try adjusting target pose.");
    }

    // 添加对象->目标规划->执行
    // 创建一个 PlanningSceneInterface 对象，并将碰撞对象应用到规划场景中
    PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_object);
    // 等待一段时间，确保碰撞对象被添加到规划场景中
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose);
    MoveGroupInterface::Plan plan_with_obstacle;
    auto const ok_with_obstacle = static_cast<bool>(arm.plan(plan_with_obstacle));
    if (ok_with_obstacle) {
      RCLCPP_INFO(node->get_logger(), "Plan succeeded. Executing collision-aware trajectory.");
      auto const exec_ok = static_cast<bool>(arm.execute(plan_with_obstacle));
      RCLCPP_INFO(node->get_logger(), "Execute result: %s", exec_ok ? "success" : "failed");
    } else {
      RCLCPP_WARN(node->get_logger(), "Plan failed. Try adjusting obstacle size/pose.");
    }

    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
    return 0;
}