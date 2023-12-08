#ifndef COMPOSITION_PREAPPROACH_COMPONENT_HPP_
#define COMPOSITION_PREAPPROACH_COMPONENT_HPP_


#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/utilities.hpp"
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include <memory>

#include "my_components/visibility_control.h"

namespace my_components
{

class PreApproach : public rclcpp::Node
{
    public:
        COMPOSITION_PUBLIC
        explicit PreApproach(const rclcpp::NodeOptions & options);
     

    private:
    
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
       
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        void rotate_robot();
        


    //defining variables
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_ ;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        
        geometry_msgs::msg::Twist vel_msg;
        //nav_msgs::msg::Odometry odom_msg;



        int front_laser_array_index;
        float front_laser_reading;
        float obstacle_distance_parameter = 0.45;
        float angle_to_be_rotated_parameter = -90.0;
        bool dest_reached;
        bool rotate_status;
        bool rotation_start_status;
        float target_angle;
        float angular_speed = 0.4;
        float translation_speed = 0.6;
        float current_yaw_angle;
        float yaw_threshold= 0.02;
};

} //namespace composition

#endif
