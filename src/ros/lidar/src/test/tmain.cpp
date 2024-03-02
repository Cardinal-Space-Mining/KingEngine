#include <iostream>

#include "lidar_api.h"


int main(int argc, char** argv) {

	std::cout << "[KE-LIDAR (test)]: Successfully established program entrypoint!" << std::endl;

	ldrp::LidarConfig _config{};

	ldrp::apiInit(_config);
	ldrp::apiDestroy();

}
