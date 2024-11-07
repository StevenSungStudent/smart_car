#ifndef SMARTCAR_SIMULATION_ODOMETRY
#define SMARTCAR_SIMULATION_ODOMETRY

#include "rclcpp/rclcpp.hpp"
#include "smartcar_msgs/msg/status.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include "rclcpp/duration.hpp"

class Odometry : public rclcpp::Node
{
  public:
    Odometry(/* args */);
    ~Odometry();

  private:
    void status_callback(const smartcar_msgs::msg::Status & msg);
    void timer_callback();

    double calculate_linear_velocity(const double& rpm, const double& wheel_diameter) const;
    double calculate_angular_velocity(const double& linear_velocity, const double& wheel_distance, const double& steering_angle) const;
    
    double calculate_phi(const double& phi_pre, const double& angular_velocity, const double& time_step) const;
    double calculate_x(const double& x_pre, const double& linear_velocity, const double& phi, const double& time_step) const;
    double calculate_y(const double& y_pre, const double& linear_velocity, const double& phi, const double& time_step) const;


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Subscription<smartcar_msgs::msg::Status>::SharedPtr status_subscription_;

    double linear_velocity;
    double angular_velocity;

    double x;
    double y;
    double phi;

    rclcpp::Time last_time;
};  

#endif