#ifndef COMPOSITION__ATTACHCLIENT_COMPONENT_HPP_
#define COMPOSITION__ATTACHCLIENT_COMPONENT_HPP_



#include "rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include "attach_shelf/srv/go_to_loading.hpp"
#include "my_components/visibility_control.h"



using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using MyCustomServiceMessage = attach_shelf::srv::GoToLoading;

namespace my_components
{

class AttachClient : public rclcpp::Node
{
    public:
        COMPOSITION_PUBLIC
        explicit AttachClient(const rclcpp::NodeOptions & options);

    private:
        //method to call the service
        void call_server();

        //callback function for service
        void service_response_callback(rclcpp::Client<MyCustomServiceMessage>::SharedFuture future);


    //defining variables
        rclcpp::Client<MyCustomServiceMessage>::SharedPtr service_client_;
        bool final_approach_parameter = true;
};

}   //namespace composition


#endif



