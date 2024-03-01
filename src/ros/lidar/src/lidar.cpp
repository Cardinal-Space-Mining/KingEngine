#include "lidar/ros_bridge.hpp"
#include "lidar_api.h"


void ros_bridge::on_pose_update(const custom_types::msg::Pose &pose) {
    const float
        _xyz[3] = { pose.x, pose.y, pose.z },
        _quat[4] = { pose.qx, pose.qy, pose.qz, pose.qw };
    ldrp::updateWorldPose(_xyz, _quat, 0);
}

void ros_bridge::on_startup() {
    ldrp::apiInit();
    ldrp::lidarInit();
}

void ros_bridge::on_shutdown() {    // may block since this waits for all threads to join
    ldrp::lidarShutdown();
    ldrp::apiDestroy();
}
