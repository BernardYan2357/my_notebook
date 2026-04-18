#include "ctrl_robot_hardware/example_motor_driver.hpp"

namespace ctrl_robot_hardware
{

ExampleMotorDriver::ExampleMotorDriver(const std::string& port)
{
    (void)port;
}

bool ExampleMotorDriver::init()
{
    return true;
}

void ExampleMotorDriver::activateWithVelocityMode(int motor_id)
{
    (void)motor_id;
}

void ExampleMotorDriver::activateWithPositionMode(int motor_id)
{
    (void)motor_id;
}

void ExampleMotorDriver::deactivate(int motor_id)
{
    (void)motor_id;
}

double ExampleMotorDriver::getVelocityRadianPerSec(int motor_id)
{
    (void)motor_id;
    return 0.0;
}

void ExampleMotorDriver::setVelocityRadianPerSec(int motor_id, double velocity)
{
    (void)motor_id;
    (void)velocity;
}

} // namespace ctrl_robot_hardware
