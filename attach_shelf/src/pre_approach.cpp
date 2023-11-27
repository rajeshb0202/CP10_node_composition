#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;


class PreApproach : public rclcpp::Node
{
    public:
        PreApproach():Node("pre_approach_node")
        {
            //defining parameters
            this->declare_parameter("obstacle", 0.0);
            this->declare_parameter("degrees", 0);

            getting_params();
            dest_reached = false;
            rotate_status = false;

            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
            subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&PreApproach::scan_callback, this, _1));
        }

    private:
        void getting_params()
        {
            obstacle_distance_parameter = this->get_parameter("obstacle").get_parameter_value().get<float>();
            angle_to_be_rotated_parameter = (this->get_parameter("degrees").get_parameter_value().get<int>()) *  (M_PI / 180.0);
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
            float rotation_time = std::abs(angle_to_be_rotated_parameter/angular_speed);
            
            //setting the angular speed according to the sign
            vel_msg.linear.x = 0.0;
            if(angle_to_be_rotated_parameter > 0)
            {
                vel_msg.angular.z = angular_speed;
            }
            else if (angle_to_be_rotated_parameter < 0)
            {
                vel_msg.angular.z = -1*angular_speed;
            }
            else {
                vel_msg.angular.z = 0;
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Rotation started!");
            rclcpp::Rate loop_rate(1000);

            auto start_time = this->now();
            while((this->now() - start_time).seconds() < rotation_time)
            {
                publisher_->publish(vel_msg);
                loop_rate.sleep();
                //std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            vel_msg.angular.z = 0.0;
            publisher_->publish(vel_msg);
            RCLCPP_INFO(this->get_logger(), "Rotation executed!");
            rotate_status = true;
        }



    //defining variables
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_ ;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist vel_msg;

    int front_laser_array_index;
    float front_laser_reading;
    float obstacle_distance_parameter;
    int angle_to_be_rotated_parameter;
    bool dest_reached;
    bool rotate_status;
    float angular_speed = 0.3;
    float translation_speed = 0.6;

};

int main (int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PreApproach>());
    rclcpp::shutdown();
    return 0;
}