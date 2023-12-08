#ifndef COMPOSITION__ATTACHSERVER_COMPONENT_HPP_
#define COMPOSITION__ATTACHSERVER_COMPONENT_HPP_


#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/empty.hpp"

#include "my_components/visibility_control.h"

using MyCustomServiceMessage = attach_shelf::srv::GoToLoading;

namespace my_components
{

class AttachServer : public rclcpp::Node  // Ensure this class name is what you intended
{
public:
    COMPOSITION_PUBLIC
    explicit AttachServer(const rclcpp::NodeOptions & options);



private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void service_callback(const std::shared_ptr<MyCustomServiceMessage::Request> request,  const std::shared_ptr<MyCustomServiceMessage::Response> response);



    void timer_callback();

    void move_underneath_the_cart();

    void elevator_up();



    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);


    void detect_table_legs();



    void middle_point_table_legs(std::vector<int> table_legs_indexes);


    //calculate x and y oordinates
    float calculate_coordinate(std::string coordinate, int index);


    //method to publish the tf frame of the cart
    void publish_tf_cart_frame();



    //variables defined
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;   
    rclcpp::Service<MyCustomServiceMessage>::SharedPtr service_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
    rclcpp::CallbackGroup::SharedPtr scan_callback_group_;
    rclcpp::SubscriptionOptions odom_subscription_options_;
    rclcpp::SubscriptionOptions scan_subscription_options_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr elevator_up_publisher_;

    sensor_msgs::msg::LaserScan::SharedPtr scan_msg_;
    geometry_msgs::msg::Twist vel_msg_;
    nav_msgs::msg::Odometry::SharedPtr odom_msg_;

    float translation_speed = 0.4; 
    float angular_speed = 0.5;
    int intensity_threshold = 8000;
    float distance_gap_threshold = 0.06;
    float distance_to_be_moved_underneath = 0.35;
    std::string parent_frame = "robot_front_laser_base_link";
    std::string child_frame = "cart_frame";
    std::string base_frame = "robot_base_link";

    
    int leg_1_index, leg_2_index;
    int number_table_legs_detected;
    float mid_point_x, mid_point_y;
    float start_x, start_y;
    float cuurent_x, cuurent_y;
    std::string x_coordinate = "x";
    std::string y_coordinate = "y";


    bool tf_listener_status;            //whether to start the tf listener or not
    bool move_robot_status;             //whether to move the robot or not
    bool publish_tf_cart_frame_status;  //whether to publish the tf frame of the cart or not
    bool moved_near_the_cart_status;    //whether the robot has moved near the cart or not
    bool moved_under_the_cart_status;   //whether the robot has moved under the cart or not
    bool first_time_moving_underneath_the_cart; //whether the robot has moved under the cart for the first time or not
    bool elevator_up_status;            //whether the elevator has moved up or not


};

} //namspace composition

#endif  //COMPOSITION__ATTACHSERVER_COMPONENT_HPP_



