#include "sampler_simple/SamplerNode.hpp"
#include "sampler_simple/SamplerSignalGenerator.hpp"

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	printf("I'm up!\n");

	printf("Getting ready to spin...\n");

	auto controlNode = std::make_shared<SamplerNode>();
	auto signalGeneratorNode = std::make_shared<SamplerSignalGeneratorNode>();

	rclcpp::executors::SingleThreadedExecutor exec;
	exec.add_node(controlNode);
	exec.add_node(signalGeneratorNode);
	
	printf("♪♪♪ Spin me right round ♪♪♪\n");  
	exec.spin();

	printf("I'm down!\n");

	return 0;
}