#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using MyCustomServiceMessage = attach_shelf::srv::GoToLoading;

class ApproachServiceServer : public rclcpp::Node  // Ensure this class name is what you intended
{
public:
    ApproachServiceServer() : Node("approach_service_server_node")  // Constructor definition corrected
    {

        number_table_legs_detected = 0;

        //laser scan subscriber
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ApproachServiceServer::scan_callback, this, _1));

        
        //tf broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd_vel", 10);

        timer_ = this->create_wall_timer(50ms, std::bind(&ApproachServiceServer::timer_callback, this));

        tf_buffer_ =  std::make_unique<tf2_ros::Buffer>(this->get_clock());
        
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }



private:
    void timer_callback()
    {
        if (tf_listener_status)
        {
            //get the transform between the cart and the robot
            geometry_msgs::msg::TransformStamped transformStamped;
            try
            {
                transformStamped = tf_buffer_->lookupTransform(base_frame, child_frame, tf2::TimePointZero);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_INFO(this->get_logger(), "tf listener failed ");
                RCLCPP_WARN(this->get_logger(), "%s", ex.what());
                return;
            }

            //get the distance between the cart and the robot
            float error_distance = sqrt(pow(transformStamped.transform.translation.x, 2) + pow(transformStamped.transform.translation.y, 2));
            float error_yaw= atan2(transformStamped.transform.translation.y, transformStamped.transform.translation.x);

            //if the distance is greater than distance gap threshold, then move the robot or else stop the robot.
            if (error_distance > distance_gap_threshold)
            {
                vel_msg_.linear.x = error_distance * translation_speed;
                vel_msg_.angular.z = error_yaw * angular_speed;
            }
            else
            {
                vel_msg_.linear.x = 0;
                vel_msg_.angular.z = 0;
            }

            vel_publisher_->publish(vel_msg_);
        }
        else
        {
            vel_msg_.linear.x = 0;
            vel_msg_.angular.z = 0;
            vel_publisher_->publish(vel_msg_);
        }
    }



    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        scan_msg_ = msg;
        detect_table_legs();
    }




    void detect_table_legs()
    {
        std::vector<int> table_legs_indexes;
        int start_angle_index = 0, end_angle_index = 0;
        bool leg_detection_going_on = false;
        number_table_legs_detected = 0;

        for (size_t i = 0; i < (scan_msg_ -> intensities.size()); i++)
        {
            if (scan_msg_ -> intensities[i] >= intensity_threshold)
            {
                if (!leg_detection_going_on)
                {
                    start_angle_index = i;   
                }
                
                end_angle_index = i;
                leg_detection_going_on = true;
            }
            else
            {
                if (leg_detection_going_on && start_angle_index != end_angle_index)
                {
                    number_table_legs_detected += 1;
                    leg_1_index = int((start_angle_index + end_angle_index)/2);
                    table_legs_indexes.push_back(leg_1_index);
                }
                leg_detection_going_on = false;
                start_angle_index = 0;
                end_angle_index = 0;
            }
        }

        // Check if the last leg was still being detected when the loop ended
        if (leg_detection_going_on && start_angle_index != end_angle_index)
        {
            number_table_legs_detected += 1;
            leg_1_index = int((start_angle_index + end_angle_index)/2);
            table_legs_indexes.push_back(leg_1_index);
        }

        //print the number of table legs detected
        RCLCPP_INFO(this->get_logger(), "number of table legs detected: %d", number_table_legs_detected);

        //if there are 2 legs detected, then find the middle point and publish the tf frame
        if (number_table_legs_detected == 2)
        {
            middle_point_table_legs(table_legs_indexes);
        }
    }



    void middle_point_table_legs(std::vector<int> table_legs_indexes)
    {
        float leg_1_x, leg_1_y, leg_2_x, leg_2_y;

        //find the angle of the table legs
        leg_1_x = calculate_coordinate(x_coordinate, table_legs_indexes[0]);
        leg_1_y = calculate_coordinate(y_coordinate, table_legs_indexes[0]);
        leg_2_x = calculate_coordinate(x_coordinate, table_legs_indexes[1]);
        leg_2_y = calculate_coordinate(y_coordinate, table_legs_indexes[1]);

        mid_point_x = (leg_1_x + leg_2_x)/2;
        mid_point_y = (leg_1_y + leg_2_y)/2;

        RCLCPP_INFO(this->get_logger(), "mid point x: %f", mid_point_x);
        RCLCPP_INFO(this->get_logger(), "mid point y: %f", mid_point_y);

        //publishing the tf frame of the cart and the robot
        publish_tf_cart_frame();
    }


    //calculate x and y oordinates
    float calculate_coordinate(std::string coordinate, int index)
    {
        float coordinate_value;
        if (coordinate == "x")
        {
            coordinate_value = scan_msg_ -> ranges[index] * cos(scan_msg_ -> angle_min + index * scan_msg_ -> angle_increment);
        }
        else if (coordinate == "y")
        {
            coordinate_value = scan_msg_ -> ranges[index] * sin(scan_msg_ -> angle_min + index *scan_msg_ -> angle_increment);
        }
        return coordinate_value;
    }


    //method to publish the tf frame of the cart
    void publish_tf_cart_frame()
    {
        //publish the tf frame
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = parent_frame;
        transformStamped.child_frame_id = child_frame;
        transformStamped.transform.translation.x = mid_point_x;
        transformStamped.transform.translation.y = mid_point_y;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(transformStamped);
        RCLCPP_INFO(this->get_logger(), "published the tf frame of the cart");

        tf_listener_status = true;
    }



    //variables defined
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;   

    sensor_msgs::msg::LaserScan::SharedPtr scan_msg_;
    geometry_msgs::msg::Twist vel_msg_;

    float translation_speed = 0.4; 
    float angular_speed = 0.5;
    int intensity_threshold = 8000;
    float distance_gap_threshold = 0.1;
    std::string parent_frame = "robot_front_laser_base_link";
    std::string child_frame = "cart_frame";
    std::string base_frame = "robot_base_link";

    bool attach_to_shelf_status; 
    int leg_1_index, leg_2_index;
    int number_table_legs_detected;
    float mid_point_x, mid_point_y;
    std::string x_coordinate = "x";
    std::string y_coordinate = "y";

    //temporary variables
    bool tf_listener_status = false;


};





int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ApproachServiceServer>());  // Ensure the class name matches
    rclcpp::shutdown();
    return 0;
}
