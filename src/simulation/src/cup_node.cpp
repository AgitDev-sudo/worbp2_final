#include "cup.hpp"

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Cup>());
    rclcpp::shutdown();
    return 0;
}