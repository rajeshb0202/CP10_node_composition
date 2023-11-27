#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp"
#include "rclcpp/publisher.hpp"
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
            this->declare_parameter("obstacle", 0.3);
            this->declare_parameter("degrees", 90.0);

            getting_params();


            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
            subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&PreApproach::scan_callback, this, _1));
        }

    private:
        void getting_params()
        {
            obstacle_distance_parameter = this->get_parameter("obstacle").get_parameter_value().get<float>();
            degree_to_be_rotated_parameter = this->get_parameter("degrees").get_parameter_value().get<float>();
        }

        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {

            front_laser_array_index= int((0 - msg->angle_min)/(msg->angle_increment));
            front_laser_reading = msg->ranges[front_laser_array_index];
            RCLCPP_INFO(this->get_logger(), "front_index: %d, front laser_reading is: %f",front_laser_array_index, front_laser_reading);

            if (front_laser_reading > obstacle_distance_parameter)
            {
               vel_msg.linear.x = 0.4;
            }
            else
            {
                vel_msg.linear.x = 0.0;
            }
            
            publisher_->publish(vel_msg);
        }




    //defining variables
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_ ;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist vel_msg;

    int front_laser_array_index;
    float front_laser_reading;
    float obstacle_distance_parameter;
    float degree_to_be_rotated_parameter;
    //float distance_gap_threshold = 0.3;

};

int main (int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PreApproach>());
    rclcpp::shutdown();
    return 0;
}