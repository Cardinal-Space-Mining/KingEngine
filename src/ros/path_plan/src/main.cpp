#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "path_plan/pathplan.hpp"


int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PathPlanNode>());
	rclcpp::shutdown();

	return 0;
}
