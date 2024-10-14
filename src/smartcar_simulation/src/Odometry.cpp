#include "Odometry.hpp"
#include <iostream>
#include "cmath"
Odometry::Odometry() : Node("odometry_node")
{
    status_subscription_ = this->create_subscription<smartcar_msgs::msg::Status>(
    "smartcar/vehicle_status", 10, std::bind(&Odometry::status_callback, this, std::placeholders::_1));
}

Odometry::~Odometry()
{
}

void Odometry::status_callback(const smartcar_msgs::msg::Status & msg) const
{
    std::cout << msg.battery_current_ma << std::endl;
    std::cout << msg.battery_percentage << std::endl;
    std::cout << msg.battery_voltage_mv << std::endl;
    std::cout << msg.engine_speed_rpm << std::endl;
    std::cout << msg.steering_angle_rad << std::endl;
}

double Odometry::calculate_linear_velocity(const double& rpm, const double& wheel_diameter) const
{
    return ((rpm * M_PI * wheel_diameter) / 60);
}

double Odometry::calculate_angular_velocity(const double& linear_velocity, const double& wheel_distance, const double& steering_angle) const
{
    return ((linear_velocity/wheel_distance)/tan(steering_angle));
}