#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rex_interfaces/msg/probe_control.hpp"

class SamplerNode : public rclcpp::Node
{
public:
    SamplerNode(rclcpp::Node::SharedPtr node) : Node("MainSampler")
    {
        subscriber_=this->create_subscription<rex_interfaces::msg::ProbeControl>("/MQTT/SamplerControl", 100, std::bind(&SamplerNode::handleInput, this, std::placeholders::_1));
        //publisher_=this->create_publisher<std_msgs::msg::String>("CAN/TX/set_motor_vel", 10);

        //auto message = std_msgs::msg::String();
        //message.data = "I LIVE";
        //publisher_->publish(message);
    }
private:
   rclcpp::Subscription<rex_interfaces::msg::ProbeControl>::SharedPtr subscriber_;
   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

   void handleInput(rex_interfaces::msg::ProbeControl probleControlMessage);
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto n = rclcpp::Node::make_shared("sampler_simple");
    
    printf("I'm up\n");

    SamplerNode sampler(n);

    rclcpp::spin(n);

    printf("I'm down\n");

    return 0;
}

void SamplerNode::handleInput(rex_interfaces::msg::ProbeControl probleControlMessage)
{
    printf("I did something");
}