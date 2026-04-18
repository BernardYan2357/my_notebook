#ifndef MOBILE_BASE_HARDWARE_INTERFACE_HPP
#define MOBILE_BASE_HARDWARE_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp>
#include <vector>
#include "ctrl_robot_hardware/example_motor_driver.hpp"

namespace ctrl_robot_hardware
{

class MobileBaseHardwareInterface : public hardware_interface::SystemInterface
{
public:
    // Lifecycle node interface overrides
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    
    // System interface overrides
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    void reset_state();

    std::shared_ptr<ExampleMotorDriver> driver_;
    int left_motor_id_;
    int right_motor_id_;
    std::string port_;

    double left_wheel_pos_ = 0.0;
    double right_wheel_pos_ = 0.0;
    double left_wheel_vel_ = 0.0;
    double right_wheel_vel_ = 0.0;
    double left_wheel_cmd_ = 0.0;
    double right_wheel_cmd_ = 0.0;
}; 

} // namespace ctrl_robot_hardware

#endif // MOBILE_BASE_HARDWARE_INTERFACE_HPP