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
        approach_service_ = this->create_service<MyCustomServiceMessage>("approach_shelf", 
                      std::bind(&AprroachServiceServer::service_callback, this, _1, _2));  // Use the correct class name here
    }

private:
    void service_callback(const std::shared_ptr<MyCustomServiceMessage::Request> request, 
                          const std::shared_ptr<MyCustomServiceMessage::Response> response)
    {
        attach_to_shelf_value = request->attach_to_shelf;
        RCLCPP_INFO(this->get_logger(), "attach to shelf value is: %s", 
                    attach_to_shelf_value ? "true" : "false");

        response->complete = true;
    }

    rclcpp::Service<MyCustomServiceMessage>::SharedPtr approach_service_;
    bool attach_to_shelf_value;  
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AprroachServiceServer>());  // Ensure the class name matches
    rclcpp::shutdown();
    return 0;
}
