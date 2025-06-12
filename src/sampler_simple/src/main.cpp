#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rex_interfaces/msg/probe_control.hpp"
#include "rex_interfaces/msg/probe_status.hpp"
#include "rex_interfaces/msg/rover_status.hpp"
#include "rex_interfaces/msg/sampler_control.hpp"
#include "libVescCan/VESC.h"

class SamplerNode : public rclcpp::Node
{
public:
    SamplerNode()
    : Node("control_subscriber")
    {
        mqttSamplerControlSub_ = this->create_subscription<rex_interfaces::msg::ProbeControl>(
            "/MQTT/SamplerControl", 10,
            std::bind(&SamplerNode::control_callback, this, std::placeholders::_1)
        );

        mqttSamplerStatusSub_ = this->create_subscription<rex_interfaces::msg::ProbeStatus>(
            "/CAN/RX/probe_status", 10,
            std::bind(&SamplerNode::sampler_status_callback, this, std::placeholders::_1)
        );

        mqttRoverStatusSub_ = this->create_subscription<rex_interfaces::msg::RoverStatus>(
            "/MQTT/RoverStatus", 10,
            std::bind(&SamplerNode::rover_status_callback, this, std::placeholders::_1)
        );
        
        publisher_ = this->create_publisher<rex_interfaces::msg::SamplerControl>(
            "/CAN/TX/sampler_control", 10
        );
    }

private:
    void control_callback(const rex_interfaces::msg::ProbeControl::SharedPtr msg) const
    {
        printf("I received a message\n");
        printf("ProbeControl: %f\n",
               msg->container_degrees_0);

        auto response = rex_interfaces::msg::SamplerControl();
        response.axes[0].command_id = VESC_COMMAND_SET_DUTY;
        response.axes[0].set_value = msg->platform_movement;     //0x80

        response.axes[1].command_id = VESC_COMMAND_SET_DUTY;
        response.axes[1].set_value = msg->drill_movement;     //0x81
    
        response.axes[2].command_id = VESC_COMMAND_SET_DUTY;
        response.axes[2].set_value = msg->drill_action;     //0x82
    

        //auto response = std_msgs::msg::String();
        //response.axes[0] = msg->drill_action;
        publisher_->publish(response);
    }
    
    void sampler_status_callback(const rex_interfaces::msg::ProbeStatus::SharedPtr msg) const
    {
        printf("I received a message on sampler_status.\n");
        printf("Current distance is: %.2f.\n",
               msg->distance);
        
    }

    void rover_status_callback(const rex_interfaces::msg::RoverStatus::SharedPtr msg) const
    {
        printf("I received a message on rover_status\n");
        
    }

    rclcpp::Subscription    <rex_interfaces::msg::ProbeControl>     ::SharedPtr mqttSamplerControlSub_;
    rclcpp::Subscription    <rex_interfaces::msg::ProbeStatus>      ::SharedPtr mqttSamplerStatusSub_;
    rclcpp::Subscription    <rex_interfaces::msg::RoverStatus>      ::SharedPtr mqttRoverStatusSub_;
    rclcpp::Publisher       <rex_interfaces::msg::SamplerControl>   ::SharedPtr publisher_;

    const float minDistance_ = 5.0f; // Distance (in cm) to the ground at which the sampler movement should be stopped
    float distance_ = 100.0f;

};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    printf("I'm up\n");

    printf("starting to spin");
    rclcpp::spin(std::make_shared<SamplerNode>());

    printf("I'm down\n");

    return 0;
}