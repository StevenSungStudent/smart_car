#include "JointStatePublisher.hpp"
#include <iostream>
#include "cmath"

using namespace std::chrono_literals;

enum joints_index{
    front_left_wheel_steer_joint = 0,
    front_left_wheel_joint = 1,
    front_right_wheel_steer_joint = 2,
    front_right_wheel_joint = 3,
    back_left_wheel_joint = 4,
    back_right_wheel_joint = 5
};

JointStatePublisher::JointStatePublisher() : Node("joint_state_publisher")
{
    joint_states.name = {"front_left_wheel_steer_joint", "front_left_wheel_joint", "front_right_wheel_steer_joint", "front_right_wheel_joint", "back_left_wheel_joint", "back_right_wheel_joint"};
    joint_states.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    joint_states.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    timer_ = this->create_wall_timer( 500ms, std::bind(&JointStatePublisher::joint_publisher_callback, this));

    status_subscription_ = this->create_subscription<smartcar_msgs::msg::Status>("smartcar/vehicle_status", 10, std::bind(&JointStatePublisher::status_callback, this, std::placeholders::_1));
    publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
}

JointStatePublisher::~JointStatePublisher()
{

}

//TODO setting the velocity is not enought to make the wheels turn they also need a position to turn towards.
void JointStatePublisher::joint_publisher_callback()
{
    joint_states.header.stamp = this->get_clock()->now();
    publisher->publish(joint_states);
}

void JointStatePublisher::status_callback(const smartcar_msgs::msg::Status & msg)
{
    double radians_delta = (msg.engine_speed_rpm * ((2 * M_PI) / 60)) * 0.5;//rpm * ((2 * pi) / 60) = radiance per second * time(the wall timer value) = radiance delta.

    for (unsigned short i = 0; i < joint_states.velocity.size(); ++i)
    {
        joint_states.position.at(i) += radians_delta;
    }

    joint_states.position.at(front_left_wheel_steer_joint) = msg.steering_angle_rad;
    joint_states.position.at(front_right_wheel_steer_joint) = msg.steering_angle_rad;
}