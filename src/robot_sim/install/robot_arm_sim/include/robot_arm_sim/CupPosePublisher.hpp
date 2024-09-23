#ifndef CUP_POSE_PUBLISHER_HPP
#define CUP_POSE_PUBLISHER_HPP

#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_msgs/msg/tf_message.h"
#include "tf2/exceptions.h"


class CupPosePublisher : public rclcpp::Node
{
public:
    CupPosePublisher();
    ~CupPosePublisher();

private:
    /**
     * @brief This function publishes the postion of the cup every time it is called, in this case 10 times per second since that is the set update frequency.
     * 
     */
    void pose_publisher_callback();

    /**
     * @brief This function reads the tf data from the simulation. It reads the postion of the grippers and and of the robot arm.
     * 
     */
    void parse_transform_data();

    /**
     * @brief This function calculates the new position of the cup based on the movement of the robot arm.
     * 
     */
    void update_cup_position();


    /**
     * @brief This function calculates the distance of two points in 2d space.
     * 
     * @param x1 x of the first point.
     * @param y1 y of the first point.
     * @param x2 x of the second point.
     * @param y2 y of the second point.
     * @return The distance between the points in a double. 
     */
    double distance_2d(double x1, double y1, double x2, double y2);

    /**
     * @brief The broadcaster for publishing the cup position.
     * 
     */
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /**
     * @brief The timer for publishing.
     * 
     */
    rclcpp::TimerBase::SharedPtr timer;

    /**
     * @brief The listener for reading tf data.
     * 
     */
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    /**
     * @brief A buffer where the read tf data is stored.
     * 
     */
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    /**
     * @brief Data about the position of the hand.
     * 
     */
    geometry_msgs::msg::Transform hand_position;

    /**
     * @brief Data about the position of the left gripper.
     * 
     */
    geometry_msgs::msg::Transform gripper_left_position;

    /**
     * @brief Data about the position of the right gripper.
     * 
     */
    geometry_msgs::msg::Transform gripper_right_position;

    /**
     * @brief The current position of the cup.
     * 
     */
    geometry_msgs::msg::TransformStamped current_pose;

    /**
     * @brief The frequency at which the cup position will be published.
     * 
     */
    unsigned short update_frequency;

    /**
     * @brief The const value of gravity.
     * 
     */
    const double gravity = 9.81;

};

#endif // CUP_POSE_PUBLISHER_HPP
