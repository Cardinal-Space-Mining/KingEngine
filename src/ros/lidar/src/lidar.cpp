#include "lidar/ros_bridge.hpp"
#include "lidar_api.h"


void ros_bridge::on_pose_update(const custom_types::msg::Pose& pose, uint64_t ts_us) {

    const float _pose[7] =
    {
        (float)pose.x,
        (float)pose.y,
        (float)pose.z,
        (float)pose.qx,
        (float)pose.qy,
        (float)pose.qz,
        (float)pose.qw
    };

    ldrp::updateWorldPose(_pose, _pose + 3, ts_us);     // << timestamp!?

}

void ros_bridge::on_startup() {

    ldrp::apiInit();
    ldrp::lidarInit();

}
void ros_bridge::on_shutdown() {    // may block since this waits for all threads to join

    ldrp::lidarShutdown();
    ldrp::apiDestroy();

}
