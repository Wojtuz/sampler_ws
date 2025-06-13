#include "sampler_simple/SamplerNode.hpp"

SamplerNode::SamplerNode() : Node("sampler_node")
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
	client_ = this->create_client<rex_interfaces::srv::SamplerSetGenerator>("set_generator");
}

void SamplerNode::control_callback(const rex_interfaces::msg::ProbeControl::SharedPtr msg)
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
			response.servos[0].set_value = msg->container_degrees_0; 
			response.servos[1].set_value = msg->container_degrees_1; 
			response.servos[2].set_value = msg->container_degrees_2;
		}
		else
		{
			printf("Platform is too low to move servos. Using last good response.\n");
			response.servos[0].set_value = lastGoodResponse_.servos[0].set_value; 
			response.servos[1].set_value = lastGoodResponse_.servos[1].set_value;
			response.servos[2].set_value = lastGoodResponse_.servos[2].set_value;
		}

		response.header.stamp = msg->header.stamp;
		lastGoodResponse_ = response;

		auto request = std::make_shared<rex_interfaces::srv::SamplerSetGenerator::Request>();
			request->response = response;

		client_->async_send_request(request);

		show_requested_vs_published(msg, response);
	}

void SamplerNode::sampler_status_callback(const rex_interfaces::msg::ProbeStatus::SharedPtr msg)
	{
		printf("I received a message on sampler_status.\n");
		printf("Current distance is: %.2f.\n",
				msg->distance);
        
		currentDistance_ = msg->distance;
	}

void SamplerNode::rover_status_callback(const rex_interfaces::msg::RoverStatus::SharedPtr msg)
{
	printf("I received a message on rover_status, %i\n",
			msg->control_mode);
	if(msg->control_mode == currentControlMode)
	{
		printf("Control mode didn't change.\n");
		currentControlMode = msg->control_mode;
		return;
	}
	if(currentControlMode == 3 && msg->control_mode != 3)
	{
		printf("Control mode changed from sampler to something else.\n");
		auto request = std::make_shared<rex_interfaces::srv::SamplerSetGenerator::Request>();
			
		lastGoodResponse_.axes[0].set_value = 0.0f; //stop platform movement
		lastGoodResponse_.axes[1].set_value = 0.0f; //stop drill movement
		lastGoodResponse_.axes[2].set_value = 0.0f; //stop drill action
		lastGoodResponse_.header.stamp = this->now();

		request->response = lastGoodResponse_;
		client_->async_send_request(request);
	}
}

void SamplerNode::show_requested_vs_published(const rex_interfaces::msg::ProbeControl::SharedPtr msg,
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

float SamplerNode::handle_platform_movement(float platform_movement) const
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
