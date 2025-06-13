#include "rclcpp/rclcpp.hpp"
#include "rex_interfaces/msg/sampler_control.hpp"
#include "rex_interfaces/srv/sampler_set_generator.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

class SamplerSignalGeneratorNode : public rclcpp::Node
{
public:
	SamplerSignalGeneratorNode();

private:
	void timer_callback();

	void on_service_called(const std::shared_ptr<rex_interfaces::srv::SamplerSetGenerator::Request> request,
		std::shared_ptr<rex_interfaces::srv::SamplerSetGenerator::Response> response);

	rclcpp::Service         <rex_interfaces::srv::SamplerSetGenerator>  ::SharedPtr service_;
	rclcpp::Publisher       <rex_interfaces::msg::SamplerControl>       ::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

	rex_interfaces::msg::SamplerControl lastGoodResponse_ = rex_interfaces::msg::SamplerControl();
};