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

using namespace std::chrono_literals;
using std::placeholders::_1;


class PreApproach : public rclcpp::Node
{
    public:
        PreApproach():Node("pre_approach_node")
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

    private:
    
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            tf2::Quaternion quat;
            tf2::fromMsg(msg->pose.pose.orientation, quat);

            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            current_yaw_angle = yaw; 
        }



        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            front_laser_array_index= int((0 - msg->angle_min)/(msg->angle_increment));
            front_laser_reading = msg->ranges[front_laser_array_index];

    
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




        void rotate_robot()
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


            RCLCPP_INFO(this->get_logger(), "Rotation started...");
            rclcpp::Rate loop_rate(100);
            
            if (!rotation_start_status)
            {
                target_angle = current_yaw_angle + (angle_to_be_rotated_parameter * M_PI/180);
                rotation_start_status = true;
            }

            if (fabs(target_angle - current_yaw_angle) > yaw_threshold)
            {
                publisher_->publish(vel_msg);
                RCLCPP_INFO(this->get_logger(), "target_angle: %.1f, current_angle: %.1f", target_angle, current_yaw_angle );
                loop_rate.sleep();
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



    //defining variables
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_ ;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        
        geometry_msgs::msg::Twist vel_msg;
        //nav_msgs::msg::Odometry odom_msg;

        
        rclcpp::CallbackGroup::SharedPtr odom_callback_group;
        rclcpp::CallbackGroup::SharedPtr scan_callback_group;


        int front_laser_array_index;
        float front_laser_reading;
        float obstacle_distance_parameter = 0.3;
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




int main (int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto pre_approach_node_obj = std::make_shared<PreApproach>();

    rclcpp::spin(pre_approach_node_obj);

    rclcpp::shutdown();
    return 0;
}