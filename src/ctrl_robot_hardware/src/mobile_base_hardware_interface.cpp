#include "ctrl_robot_hardware/mobile_base_hardware_interface.hpp"

namespace ctrl_robot_hardware
{

namespace
{
constexpr const char* kLeftWheelJoint = "base_left_wheel_joint";
constexpr const char* kRightWheelJoint = "base_right_wheel_joint";
constexpr const char* kIfPosition = "position";
constexpr const char* kIfVelocity = "velocity";
} // namespace

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_init
    (const hardware_interface::HardwareInfo& hardware_info)
{
    if (this->hardware_interface::SystemInterface::on_init(hardware_info) != 
        hardware_interface::CallbackReturn::SUCCESS){
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    info_ = hardware_info; // info_ 是 SystemInterface 的私有属性
    left_motor_id_ =  10;
    right_motor_id_ = 20;
    port_ = "/dev/ttyUSB0";
    driver_ = std::make_shared<ExampleMotorDriver>(port_);
    reset_state();
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MobileBaseHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(kLeftWheelJoint, kIfPosition, &left_wheel_pos_);
    state_interfaces.emplace_back(kLeftWheelJoint, kIfVelocity, &left_wheel_vel_);
    state_interfaces.emplace_back(kRightWheelJoint, kIfPosition, &right_wheel_pos_);
    state_interfaces.emplace_back(kRightWheelJoint, kIfVelocity, &right_wheel_vel_);

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MobileBaseHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(kLeftWheelJoint, kIfVelocity, &left_wheel_cmd_);
    command_interfaces.emplace_back(kRightWheelJoint, kIfVelocity, &right_wheel_cmd_);

    return command_interfaces;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_configure
    (const rclcpp_lifecycle::State& previous_state)
{
    (void)previous_state;
    if (driver_->init() != false) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_activate
    (const rclcpp_lifecycle::State& previous_state)
{
    (void)previous_state;
    driver_->activateWithVelocityMode(left_motor_id_);
    driver_->activateWithVelocityMode(right_motor_id_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_deactivate
    (const rclcpp_lifecycle::State& previous_state)
{
    (void)previous_state;
    driver_->deactivate(left_motor_id_);
    driver_->deactivate(right_motor_id_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MobileBaseHardwareInterface::read
    (const rclcpp::Time& time, const rclcpp::Duration& period) // period 指的是上一次控制周期的持续时间
{
    (void)time;
    left_wheel_vel_ = driver_->getVelocityRadianPerSec(left_motor_id_);
    right_wheel_vel_ = driver_->getVelocityRadianPerSec(right_motor_id_);

    const double dt = period.seconds();
    left_wheel_pos_ += left_wheel_vel_ * dt;
    right_wheel_pos_ += right_wheel_vel_ * dt;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MobileBaseHardwareInterface::write
    (const rclcpp::Time& time, const rclcpp::Duration& period)
{
    (void)time;
    (void)period;
    driver_->setVelocityRadianPerSec(left_motor_id_, left_wheel_cmd_);
    driver_->setVelocityRadianPerSec(right_motor_id_, right_wheel_cmd_);
    return hardware_interface::return_type::OK;
}

void MobileBaseHardwareInterface::reset_state()
{
    left_wheel_pos_ = 0.0;
    right_wheel_pos_ = 0.0;
    left_wheel_vel_ = 0.0;
    right_wheel_vel_ = 0.0;
    left_wheel_cmd_ = 0.0;
    right_wheel_cmd_ = 0.0;
}

} // namespace ctrl_robot_hardware