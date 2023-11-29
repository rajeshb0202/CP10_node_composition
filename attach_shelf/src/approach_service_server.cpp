#include "rclcpp/rclcpp.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using MyCustomServiceMessage = attach_shelf::srv::GoToLoading;

class AprroachServiceServer : public rclcpp::Node  // Ensure this class name is what you intended
{
public:
    AprroachServiceServer() : Node("approach_service_server_node")  // Constructor definition corrected
    {

        number_table_legs_detected = 0;

        //laser scan subscriber
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&AprroachServiceServer::scan_callback, this, _1));

    }

private:
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

        for (size_t i = 0; i < (scan_msg_ -> intensities.size()) - 2; i++)
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
                if (leg_detection_going_on)
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
        if (leg_detection_going_on)
        {
            number_table_legs_detected += 1;
            leg_1_index = int((start_angle_index + end_angle_index)/2);
            table_legs_indexes.push_back(leg_1_index);
        }

        //print the number of table legs detected
        RCLCPP_INFO(this->get_logger(), "number of table legs detected: %d", number_table_legs_detected);


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




    //variables defined
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;

    sensor_msgs::msg::LaserScan::SharedPtr scan_msg_;
    bool attach_to_shelf_status;  
    int intensity_threshold = 8000;
    int leg_1_index, leg_2_index;
    int number_table_legs_detected;
    float mid_point_x, mid_point_y;
    std::string x_coordinate = "x";
    std::string y_coordinate = "y";

};





int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AprroachServiceServer>());  // Ensure the class name matches
    rclcpp::shutdown();
    return 0;
}
