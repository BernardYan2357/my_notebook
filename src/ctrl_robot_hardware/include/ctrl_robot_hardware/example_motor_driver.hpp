#ifndef EXAMPLE_MOTOR_DRIVER_HPP
#define EXAMPLE_MOTOR_DRIVER_HPP

#include <string>

namespace ctrl_robot_hardware
{

class ExampleMotorDriver
{
public:
    ExampleMotorDriver(const std::string& port);

    bool init();

    void activateWithVelocityMode(int motor_id);
    void activateWithPositionMode(int motor_id);
    void deactivate(int motor_id);

    double getVelocityRadianPerSec(int motor_id); // 返回电机当前的速度，单位为弧度每秒
    void setVelocityRadianPerSec(int motor_id, double velocity);
private:
};

}

#endif // EXAMPLE_MOTOR_DRIVER_HPP