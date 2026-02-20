#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mutex>

using JointState = sensor_msgs::msg::JointState;
using namespace std::placeholders;

class jointsSubscriber
{
public:
    jointsSubscriber(std::shared_ptr<rclcpp::Node> node)
      : node_(node),
        joint_states_sub_(node->create_subscription<JointState>(
            "joint_states", 10, std::bind(&jointsSubscriber::jointStatesCallback, this, _1))){}

    void jointStatesCallback(const JointState &msg)
    {
        // 开始临界区：lock_guard 获取 mutex
        std::lock_guard<std::mutex> lock(state_mutex_);
        // 保存关节名和位置的对应关系到 map
        for (size_t i = 0; i < msg.name.size(); ++i)
        {
            joint_positions_[msg.name[i]] = msg.position[i];
            RCLCPP_INFO(node_->get_logger(), "Joint: %s, Position: %f", 
                        msg.name[i].c_str(), msg.position[i]);
        }
        // lock_guard 析构时自动释放 mutex
    }

    double getJointPosition(const std::string &joint_name)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (joint_positions_.find(joint_name) != joint_positions_.end())
            return joint_positions_[joint_name];
        return 0.0;
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::Subscription<JointState>> joint_states_sub_;
    std::map<std::string, double> joint_positions_;  // 存储 关节名 -> 关节角度
    std::mutex state_mutex_;  // 互斥锁，保证线程安全

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("jointsSubscriber");
    jointsSubscriber subscriber(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

/*
/joint_states
type: sensor_msgs/msg/JointState

std_msgs/Header header
        builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
        string frame_id
string[] name
float64[] position
float64[] velocity
float64[] effort

header:
  stamp:
    sec: 1771597463
    nanosec: 650114795
  frame_id: base_link
name:
- joint2
- joint3
- joint1
- joint4
- joint5
- joint6
- gripper_left_finger_joint
position:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
velocity:
- .nan x6
effort:
- .nan x6
*/