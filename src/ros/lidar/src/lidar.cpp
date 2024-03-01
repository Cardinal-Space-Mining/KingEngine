#include "lidar/ros_bridge.hpp"
#include "lidar_api.h"


void ros_bridge::on_location_update(const custom_types::msg::Location &loc) {
    (void) loc;
    ldrp::updateWorldPose(nullptr, nullptr, 0.f, 0U);
}

void ros_bridge::on_startup() {
    ldrp::apiInit();
    ldrp::lidarInit();
}

void ros_bridge::on_shutdown() {    // may block since this waits for all threads to join
    ldrp::lidarShutdown();
    ldrp::apiDestroy();
}
