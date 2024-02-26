#include <iostream>

#include "lidar_api.h"


int main(int argc, char** argv) {

	std::cout << "[KE-LIDAR (test)]: Successfully established program entrypoint!" << std::endl;

	ldrp::apiInit("", "test_lidar.wpilog");
	ldrp::apiDestroy();

}
