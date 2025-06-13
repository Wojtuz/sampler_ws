#include "rclcpp/rclcpp.hpp"
#include "rex_interfaces/msg/probe_control.hpp"
#include "rex_interfaces/msg/probe_status.hpp"
#include "rex_interfaces/msg/rover_status.hpp"
#include "rex_interfaces/msg/sampler_control.hpp"
#include "rex_interfaces/srv/sampler_set_generator.hpp"
#include "libVescCan/VESC.h"

class SamplerNode : public rclcpp::Node
{
public:
	SamplerNode();

private:
	void control_callback(const rex_interfaces::msg::ProbeControl::SharedPtr msg);
    
	void sampler_status_callback(const rex_interfaces::msg::ProbeStatus::SharedPtr msg);

	void rover_status_callback(const rex_interfaces::msg::RoverStatus::SharedPtr msg);

	void show_requested_vs_published(const rex_interfaces::msg::ProbeControl::SharedPtr msg,
									const rex_interfaces::msg::SamplerControl &response) const;

	float handle_platform_movement(float platform_movement) const;

	rclcpp::Client			<rex_interfaces::srv::SamplerSetGenerator>	::SharedPtr client_;
	rclcpp::Subscription	<rex_interfaces::msg::ProbeControl>			::SharedPtr mqttSamplerControlSub_;
	rclcpp::Subscription	<rex_interfaces::msg::ProbeStatus>			::SharedPtr mqttSamplerStatusSub_;
	rclcpp::Subscription	<rex_interfaces::msg::RoverStatus>			::SharedPtr mqttRoverStatusSub_;
	rclcpp::Publisher		<rex_interfaces::msg::SamplerControl>		::SharedPtr publisher_;

	const float lowerMovementLimit_ = 5.0f; 
	const float upperMovementLimit_ = 100.0f;
	const float safeServoDistance_ = 50.0f;
	float currentDistance_ = 70.0f;
	int currentControlMode = 0;     //3 - sampler

	rex_interfaces::msg::SamplerControl lastGoodResponse_ = rex_interfaces::msg::SamplerControl();

};
