#include "sampler_simple/SamplerSignalGenerator.hpp"

SamplerSignalGeneratorNode::SamplerSignalGeneratorNode() : Node("control_signal_generator")
{
	publisher_ = this->create_publisher<rex_interfaces::msg::SamplerControl>(
		"/CAN/TX/sampler_control", 10
	);

	timer_ = this->create_wall_timer(
	    std::chrono::milliseconds(100),
	    std::bind(&SamplerSignalGeneratorNode::timer_callback, this)
	);

	service_ = this->create_service<rex_interfaces::srv::SamplerSetGenerator>(
	    "set_generator",
	    std::bind(&SamplerSignalGeneratorNode::on_service_called, this, 
        std::placeholders::_1, std::placeholders::_2)
	);
}

void SamplerSignalGeneratorNode::timer_callback()
{
	if( lastGoodResponse_.axes[0].set_value + 
		lastGoodResponse_.axes[1].set_value + 
		lastGoodResponse_.axes[2].set_value == 0) return; //Just don't resend "do nothing" command 

	lastGoodResponse_.header.stamp = this->now();
	publisher_->publish(lastGoodResponse_);
}

void SamplerSignalGeneratorNode::on_service_called(const std::shared_ptr<rex_interfaces::srv::SamplerSetGenerator::Request> request,
		std::shared_ptr<rex_interfaces::srv::SamplerSetGenerator::Response> response)
{
	lastGoodResponse_ = request->response;
	publisher_->publish(lastGoodResponse_);
}