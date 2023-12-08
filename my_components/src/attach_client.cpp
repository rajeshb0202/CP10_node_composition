
#include "rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include "attach_shelf/srv/go_to_loading.hpp"

#include "my_components/attach_client.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using MyCustomServiceMessage = attach_shelf::srv::GoToLoading;



namespace my_components
{
  
        AttachClient::AttachClient(const rclcpp::NodeOptions & options):Node("pre_approach_node")
        {
            //service client
            service_client_ = this->create_client<MyCustomServiceMessage>("/approach_shelf");
           
            RCLCPP_INFO(this->get_logger(), "client node has started!...");
            this->call_server();
        }      


        //method to call the service
        void AttachClient::call_server()
        {
            auto request = std::make_shared<MyCustomServiceMessage::Request>();
            request->attach_to_shelf = final_approach_parameter;

            while(!service_client_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {   
                    RCLCPP_INFO(this->get_logger(), "Interrupted while waiting for the service!");
                }
                RCLCPP_INFO(this->get_logger(), "the servic is not available.. waiting!");
            }

            auto result_future = service_client_->async_send_request(request, std::bind(&AttachClient::service_response_callback, this, std::placeholders::_1));
        }

        //callback function for service
        void AttachClient::service_response_callback(rclcpp::Client<MyCustomServiceMessage>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready) 
            {
                RCLCPP_INFO(this->get_logger(), "Service called successfully and its response: %s", future.get()->complete? "true" : "false");
            } 
            else 
            {
                RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
            }
        }


} //namespace 




#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)



