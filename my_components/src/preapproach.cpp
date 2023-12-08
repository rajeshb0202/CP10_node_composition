#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/utilities.hpp"
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <memory>

#include "my_components/preapproach.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace my_components
{

    PreApproach::PreApproach(const rclcpp::NodeOptions & options):Node("pre_approach_node", options)
    {
        dest_reached = false;
        rotate_status = false;
        rotation_start_status = false;


        //creating subscribers and publishers
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&PreApproach::scan_callback, this, _1));
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/diffbot_base_controller/odom", 10, std::bind(&PreApproach::odom_callback, this, _1));
        
        RCLCPP_INFO(this->get_logger(), "pre_approach node has started!...");
    }

    
    void PreApproach::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        tf2::Quaternion quat;
        tf2::fromMsg(msg->pose.pose.orientation, quat);

        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        current_yaw_angle = yaw; 
    }



    void PreApproach::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        front_laser_array_index= int((0 - msg->angle_min)/(msg->angle_increment));
        front_laser_reading = msg->ranges[front_laser_array_index];
        //for debugging
        //RCLCPP_INFO(this->get_logger(), "front lsr: %.2f", front_laser_reading);

        if (front_laser_reading > obstacle_distance_parameter  && !dest_reached)
        {
            vel_msg.linear.x = translation_speed;
        }
        else
        {
            vel_msg.linear.x = 0.0;
            dest_reached = true;
        }
        
        publisher_->publish(vel_msg);

        if (dest_reached && !rotate_status)
        {
            rotate_robot();
        }
    }




    void PreApproach::rotate_robot()
    {
        //setting the velocities for rotation
        vel_msg.linear.x = 0;
        if (angle_to_be_rotated_parameter > 0)
        {
            vel_msg.angular.z = angular_speed;
        }
        else if( angle_to_be_rotated_parameter < 0)
        {
            vel_msg.angular.z = -1 * angular_speed;
        }
        else
        {
            rotate_status = true;
            return;
        }

        rclcpp::Rate loop_rate(100);
        
        if (!rotation_start_status)
        {
            target_angle = current_yaw_angle + (angle_to_be_rotated_parameter * M_PI/180);
            rotation_start_status = true;
        }

        if (fabs(target_angle - current_yaw_angle) > yaw_threshold)
        {
            publisher_->publish(vel_msg);
            //for debugging, pritning the target angle and the current angle.
            //RCLCPP_INFO(this->get_logger(), "target_angle: %.1f, current_angle: %.1f", target_angle, current_yaw_angle );
            //loop_rate.sleep();
        }

        else
        {
            //stopping the robot after rotation
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = 0;
            publisher_->publish(vel_msg);
            rotate_status = true;
            RCLCPP_INFO(this->get_logger(), "Rotation of %0.2f degrees successfully done...", angle_to_be_rotated_parameter);
        }
    }

}



#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)


