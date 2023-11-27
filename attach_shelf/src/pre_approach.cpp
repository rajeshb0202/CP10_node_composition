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
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;


class PreApproach : public rclcpp::Node
{
    public:
        PreApproach():Node("pre_approach_node")
        {

            RCLCPP_INFO(this->get_logger(), "pre_approach node has started!...");
            //defining parameters
            this->declare_parameter("obstacle", 0.0);
            this->declare_parameter("degrees", 0);

            //intialising the variables
            getting_params();
            dest_reached = false;
            rotate_status = false;

            //callback groups
            odom_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            scan_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            //creating subscriptions options
            rclcpp::SubscriptionOptions odom_options;
            odom_options.callback_group = odom_callback_group;

            rclcpp::SubscriptionOptions scan_options;
            scan_options.callback_group = scan_callback_group;


            //creating subscribers and publishers
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
            scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&PreApproach::scan_callback, this, _1), odom_options);
            odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&PreApproach::odom_callback, this, _1), scan_options);
            
            RCLCPP_INFO(this->get_logger(), "cmd_vel publisher intialised!...");
        }

    private:
        void getting_params()
        {
            obstacle_distance_parameter = this->get_parameter("obstacle").get_parameter_value().get<float>();
            angle_to_be_rotated_parameter = (this->get_parameter("degrees").get_parameter_value().get<int>()) *  (M_PI / 180.0);
        }



        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "inside odom callback method");
            tf2::Quaternion quat;
            tf2::fromMsg(msg->pose.pose.orientation, quat);

            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            RCLCPP_INFO(this->get_logger(), "Yaw: [%f]", yaw);

            if (dest_reached && !rotate_status)
            {
                rotate_robot();
            }
        }



        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            front_laser_array_index= int((0 - msg->angle_min)/(msg->angle_increment));
            front_laser_reading = msg->ranges[front_laser_array_index];

    
            if (front_laser_reading > obstacle_distance_parameter  && !dest_reached)
            {
               RCLCPP_INFO(this->get_logger(), "front_laser_reading: %f", front_laser_reading);
               vel_msg.linear.x = translation_speed;
            }
            else
            {
                vel_msg.linear.x = 0.0;
                dest_reached = true;
            }
            
            publisher_->publish(vel_msg);
        }




        void rotate_robot()
        {
            RCLCPP_INFO(this->get_logger(), "inside rotate method");

            //write the rotation algorithm here
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
        float obstacle_distance_parameter;
        int angle_to_be_rotated_parameter;
        bool dest_reached;
        bool rotate_status;
        float angular_speed = 0.3;
        float translation_speed = 0.6;
        float yaw_angle;
};




int main (int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto pre_approach_node_obj = std::make_shared<PreApproach>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(pre_approach_node_obj);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}