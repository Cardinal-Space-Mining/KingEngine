#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "path_plan/pathplan.hpp"


int main(int argc, char * argv[]) {

#ifdef HAVE_OPENCV
	std::cout << "OpenCV was found" << std::endl;
#else
	std::cout << "OpenCV was not found, on_lidar_data() will take a while for a large amount of data." << std::endl;
#endif

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PathPlanNode>());
	rclcpp::shutdown();

	return 0;

}
