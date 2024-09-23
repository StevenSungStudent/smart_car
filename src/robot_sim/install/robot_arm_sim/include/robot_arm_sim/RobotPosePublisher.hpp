#ifndef ROBOT_POSE_PUBLISHER_HPP
#define ROBOT_POSE_PUBLISHER_HPP

#pragma once

//Info: This code is for moving the robot sim.

#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_msgs/msg/tf_message.h"
#include "tf2/exceptions.h"


class RobotPosePublisher : public rclcpp::Node
{
public:
    RobotPosePublisher();
    ~RobotPosePublisher();

private:
    /**
     * @brief This function publishes the postion of the arm every time it is called.
     * 
     */
    void joint_publisher_callback();

    /**
     * @brief This function parses commands send to the "command" topic.
     * 
     * @param command The recieved command.
     */
    void command_callback(const std_msgs::msg::String& command);

    /**
     * @brief This function converts the received pwm to a corresponding angle.
     * 
     * @param value The pwm value.
     * @return The angle in a double.
     */
    double PWM_to_angle(const long& value) const;

    /**
     * @brief This function converts the received pwm to a corresponding distance.
     * 
     * @param value The pwm value.
     * @return The distance in meters in a double.
     */
    double PWM_to_meter(const long& value) const;

    /**
     * @brief The max pwm value.
     * 
     */
    const unsigned short max_pwm_;

    /**
     * @brief The min pwm value.
     * 
     */
    const unsigned short min_pwm_;

    /**
     * @brief The frequency at which the the robot arm state will be published
     * 
     */
    const unsigned short update_frequency_;

    /**
     * @brief The delta in angle between the current position of the joint and the requested position of the joints.
     * 
     */
    std::vector<double> delta_angle;

    /**
     * @brief The publishder for the robot arm.
     * 
     */
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;

    /**
     * @brief The timer for the robot arm.
     * 
     */
    rclcpp::TimerBase::SharedPtr timer;

    /**
     * @brief The requested state of all the joints.
     * 
     */
    sensor_msgs::msg::JointState joint_states;

    /**
     * @brief The current state of all the joints.
     * 
     */
    sensor_msgs::msg::JointState current_joint_states;

    /**
     * @brief The subscription to the command topic.
     * 
     */
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;

};

#endif