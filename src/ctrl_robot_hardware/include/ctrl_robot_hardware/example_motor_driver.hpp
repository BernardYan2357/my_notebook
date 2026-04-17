#ifndef EXAMPLE_MOTOR_DRIVER_HPP
#define EXAMPLE_MOTOR_DRIVER_HPP

#include <string>

namespace ctrl_robot_hardware
{

class ExampleMotorDriver
{
public:
    ExampleMotorDriver(const std::string& port, int left_motor_id, int right_motor_id);
    ~ExampleMotorDriver();

    bool initialize();
    bool read();
    bool write();

private:
};

}

#endif // EXAMPLE_MOTOR_DRIVER_HPP