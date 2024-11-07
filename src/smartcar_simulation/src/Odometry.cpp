#include "Odometry.hpp"
#include <iostream>
#include "cmath"


using namespace std::chrono_literals;

Odometry::Odometry() : Node("odometry_node"), linear_velocity(0), angular_velocity(0), x(0), y(0), phi(0)
{
    status_subscription_ = this->create_subscription<smartcar_msgs::msg::Status>("smartcar/vehicle_status", 10, std::bind(&Odometry::status_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/smart_car/wheel/odom", 10);
    timer_ = this->create_wall_timer( 50ms, std::bind(&Odometry::timer_callback, this));
    last_time = this->get_clock()->now();
}

Odometry::~Odometry()
{
}

void Odometry::status_callback(const smartcar_msgs::msg::Status & msg)
{
    rclcpp::Time current_time = this->get_clock()->now();
    
    rclcpp::Duration time_step = current_time - last_time;

    linear_velocity = calculate_linear_velocity(msg.engine_speed_rpm, 0.064);//wheel diameter
    angular_velocity = calculate_angular_velocity(linear_velocity, 0.257, msg.steering_angle_rad);//distance between rear and front axle
    phi = calculate_phi(phi, angular_velocity, time_step.seconds());
    x = calculate_x(x, linear_velocity, phi, time_step.seconds());
    y = calculate_y(y, linear_velocity, phi, time_step.seconds());

    last_time = current_time;
}

void Odometry::timer_callback()
{
    nav_msgs::msg::Odometry msg;
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";
    msg.header.stamp = this->get_clock()->now();

    msg.pose.covariance = {
        1.0,     0.0,     0.0,     0.0,     0.0,     0.0,   
        0.0,     1.0,     0.0,     0.0,     0.0,     0.0,   
        0.0,     0.0,     100000.0, 0.0,     0.0,     0.0,   
        0.0,     0.0,     0.0,     100000.0, 0.0,     0.0,   
        0.0,     0.0,     0.0,     0.0,     100000.0, 0.0,  
        0.0,     0.0,     0.0,     0.0,     0.0,     0.5  
    };
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0; 

    tf2::Quaternion quaternion;
    quaternion.setRPY(0,0,phi);

    msg.pose.pose.orientation = tf2::toMsg(quaternion);

    msg.twist.twist.linear.x = linear_velocity;
    msg.twist.twist.linear.y = 0;
    msg.twist.twist.linear.z = 0;

    msg.twist.twist.angular.x = 0;
    msg.twist.twist.angular.y = 0;
    msg.twist.twist.angular.z = angular_velocity;

    msg.twist.covariance = {
        0.1,     0.0,     0.0,     0.0,     0.0,     0.0,   
        0.0,     100000.0, 0.0,     0.0,     0.0,     0.0, 
        0.0,     0.0,     100000.0, 0.0,     0.0,     0.0,
        0.0,     0.0,     0.0,     100000.0, 0.0,     0.0,  
        0.0,     0.0,     0.0,     0.0,     100000.0, 0.0, 
        0.0,     0.0,     0.0,     0.0,     0.0,     0.1  
    };

    publisher_->publish(msg);
}

double Odometry::calculate_linear_velocity(const double& rpm, const double& wheel_diameter) const
{
    return ((rpm * M_PI * wheel_diameter) / 60);
}

double Odometry::calculate_angular_velocity(const double& linear_velocity, const double& wheel_distance, const double& steering_angle) const
{
    return ((linear_velocity/wheel_distance) * tan(steering_angle));
}

double Odometry::calculate_phi(const double& phi_pre, const double& angular_velocity, const double& time_step) const
{
    return (phi_pre + angular_velocity * time_step);
}
double Odometry::calculate_x(const double& x_pre, const double& linear_velocity, const double& phi, const double& time_step) const
{
    return (x_pre + linear_velocity * cos(phi) * time_step);
}
double Odometry::calculate_y(const double& y_pre, const double& linear_velocity, const double& phi, const double& time_step) const
{
    return (y_pre + linear_velocity * sin(phi) * time_step);
}
