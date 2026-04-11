#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

namespace task_constructor = moveit::task_constructor;
using PlanningSceneInterface = moveit::planning_interface::PlanningSceneInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_task"); // 定义一个全局的日志记录器

class MTCTaskNode {
public:
    // 构造函数，接受 NodeOptions 以便在 Task Constructor 中使用
    MTCTaskNode(const rclcpp::NodeOptions& options)
    {
        node_ = std::make_shared<rclcpp::Node>("mtc_task_node", options);
    }
    // 获取 NodeBaseInterface 的共享指针，以便在 Task Constructor 中使用
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface()
    {
        return node_->get_node_base_interface();
    }
    // 设置规划场景，添加一个简单的圆柱形障碍物
    void setupPlanningScene()
    {
        moveit_msgs::msg::CollisionObject object;
        object.id = "object";
        object.header.frame_id = "world";
        object.primitives.resize(1);
        object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        object.primitives[0].dimensions = { 0.1, 0.02 }; // height, radius
        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.5;
        pose.position.y = -0.25;
        pose.orientation.w = 1.0;
        object.pose = pose;
        PlanningSceneInterface().applyCollisionObject(object); // 将障碍物添加到规划场景中
    }
    // 执行任务的函数，创建任务、初始化、规划并执行
    void doTask() {
      task_ = createTask();
      task_.init();
      task_.plan(5);
      task_.execute(*task_.solutions().front());
    }
private:
    task_constructor::Task task_;
    rclcpp::Node::SharedPtr node_;
    // 创建一个简单的 Task Constructor 任务，包含当前状态、打开手、移动到抓取位置等阶段
    task_constructor::Task createTask()
    {
        task_constructor::Task task;
        task.loadRobotModel(node_);
        task.setProperty("group", "arm");
        task.setProperty("eef", "gripper");
        // interpolation_planner 用于简单的打开手阶段，sampling_planner 用于更复杂的移动阶段
        auto interpolation_planner = std::make_shared<task_constructor::solvers::JointInterpolationPlanner>();
        auto sampling_planner = std::make_shared<task_constructor::solvers::PipelinePlanner>(node_);
        // Connect 阶段需要一个 GroupPlannerVector，指定每个关节组使用哪个规划器
        task_constructor::stages::Connect::GroupPlannerVector planners = {
            { "arm", sampling_planner }
        };
        // 添加阶段：当前状态、打开手、移动到抓取位置等
        task.add(std::make_unique<task_constructor::stages::CurrentState>("current"));
        task.add(std::make_unique<task_constructor::stages::MoveTo>("open hand", interpolation_planner));
        task.add(std::make_unique<task_constructor::stages::Connect>("move to pick", planners));
        // 后续再加 GenerateGraspPose / ComputeIK / ModifyPlanningScene 等
        return task;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // 创建 MTCTaskNode 实例
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<MTCTaskNode>(options);
    MTCTaskNode mtc_task_node(options);
    // 设置规划场景并执行任务
    mtc_task_node.setupPlanningScene();
    mtc_task_node.doTask();
    // 关闭 ROS2
    rclcpp::shutdown();
    return 0;
}