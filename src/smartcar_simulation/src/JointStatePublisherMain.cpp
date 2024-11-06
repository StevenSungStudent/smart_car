#include "rclcpp/rclcpp.hpp"
#include "JointStatePublisher.hpp"

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}
