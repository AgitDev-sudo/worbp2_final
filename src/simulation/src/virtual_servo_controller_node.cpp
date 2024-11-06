#include "virtual_servo_controller.hpp"
int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualServoController>());
    rclcpp::shutdown();
    return 0;
}
