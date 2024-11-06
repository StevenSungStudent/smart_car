#ifndef SMARTCAR_SIMULATION_JOINT_STATE_PUBLISHER
#define SMARTCAR_SIMULATION_JOINT_STATE_PUBLISHER

#include "rclcpp/rclcpp.hpp"
#include "smartcar_msgs/msg/status.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/duration.hpp"

class JointStatePublisher : public rclcpp::Node
{
  public:
    JointStatePublisher(/* args */);
    ~JointStatePublisher();

  private:
    void joint_publisher_callback();
    void status_callback(const smartcar_msgs::msg::Status & msg);

    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState joint_states;
    rclcpp::Subscription<smartcar_msgs::msg::Status>::SharedPtr status_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
};  

#endif