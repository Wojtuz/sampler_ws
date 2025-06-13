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
    : Node("sampler_node")
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

        auto response = rex_interfaces::msg::SamplerControl();

        response.axes[0].command_id = VESC_COMMAND_SET_DUTY;
        response.axes[1].command_id = VESC_COMMAND_SET_DUTY;
        response.axes[2].command_id = VESC_COMMAND_SET_DUTY;
        response.servos[0].command_id = VESC_COMMAND_SET_POS;
        response.servos[1].command_id = VESC_COMMAND_SET_POS;
        response.servos[2].command_id = VESC_COMMAND_SET_POS;

        response.axes[0].set_value = handle_platform_movement(msg->platform_movement);     //0x80
        
        response.axes[1].set_value = msg->drill_movement;   // 0x81
        response.axes[2].set_value = msg->drill_action;     //0x82
        
        if(currentDistance_ > safeServoDistance_)
        {
            response.servos[0].set_value = msg->container_degrees_0; // 0x83
            response.servos[1].set_value = msg->container_degrees_1; // 0x84
            response.servos[2].set_value = msg->container_degrees_2; // 0x85
        }
        else
        {
            printf("Platform is too low to move servos. Using last good response.\n");
            response.servos[0].set_value = lastGoodResponse_.servos[0].set_value; 
            response.servos[1].set_value = lastGoodResponse_.servos[1].set_value;
            response.servos[2].set_value = lastGoodResponse_.servos[2].set_value;
        }

        response.header.stamp = msg->header.stamp;
        publisher_->publish(response);

        show_requested_vs_published(msg, response);
    }
    
    void sampler_status_callback(const rex_interfaces::msg::ProbeStatus::SharedPtr msg)
    {
        printf("I received a message on sampler_status.\n");
        printf("Current distance is: %.2f.\n",
               msg->distance);
        
        currentDistance_ = msg->distance;
    }

    void rover_status_callback(const rex_interfaces::msg::RoverStatus::SharedPtr msg) const
    {
        printf("I received a message on rover_status\n");
        
    }

    void show_requested_vs_published(const rex_interfaces::msg::ProbeControl::SharedPtr msg,
                                    const rex_interfaces::msg::SamplerControl &response) const
    {
        printf("Requested:\tPublished:\nPM: %.2f\tPM: %.2f,\nDM: %.2f,\tDM: %.2f,\nDA: %.2f,\tDA: %.2f,\nS0: %.1f,\tS0: %.1f\nS1: %.1f,\tS1: %.1f,\nS2: %.1f,\tS2: %.1f,\n",
               msg->platform_movement,
               response.axes[0].set_value,
               msg->drill_movement,
               response.axes[1].set_value,
               msg->drill_action,
               response.axes[2].set_value,
               msg->container_degrees_0,
               response.servos[0].set_value,
               msg->container_degrees_1,
               response.servos[1].set_value,
               msg->container_degrees_2,
               response.servos[2].set_value
               );
    }

    float handle_platform_movement(float platform_movement) const
    {
        if(currentDistance_ < lowerMovementLimit_ && platform_movement < 0)
        {
            printf("Platform movement prevented. Too low to move down.\n");
            return 0;
        }
        if(currentDistance_ > upperMovementLimit_ && platform_movement > 0)
        {
            printf("Platform movement prevented. Too high to move up.\n");
            return 0;
        }
        return platform_movement;
    }

    rclcpp::Subscription    <rex_interfaces::msg::ProbeControl>     ::SharedPtr mqttSamplerControlSub_;
    rclcpp::Subscription    <rex_interfaces::msg::ProbeStatus>      ::SharedPtr mqttSamplerStatusSub_;
    rclcpp::Subscription    <rex_interfaces::msg::RoverStatus>      ::SharedPtr mqttRoverStatusSub_;
    rclcpp::Publisher       <rex_interfaces::msg::SamplerControl>   ::SharedPtr publisher_;

    const float lowerMovementLimit_ = 5.0f; 
    const float upperMovementLimit_ = 100.0f;
    const float safeServoDistance_ = 50.0f;
    float currentDistance_ = 70.0f;

    rex_interfaces::msg::SamplerControl lastGoodResponse_ = rex_interfaces::msg::SamplerControl();

};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    printf("I'm up\n");

    printf("starting to spin");  
    
    auto controlNode = std::make_shared<SamplerNode>();

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(controlNode);
    exec.spin();

    printf("I'm down\n");

    return 0;
}