#include <vector>
#include <mutex>

#include "lidar/ros_bridge.hpp"
#include "lidar_api.h"


void ros_bridge::on_startup() {

    ldrp::apiInit();
    ldrp::lidarInit();

}
void ros_bridge::on_shutdown() {    // may block since this waits for all threads to join

    ldrp::lidarShutdown();
    ldrp::apiDestroy();

}

void ros_bridge::on_pose_update(const custom_types::msg::Pose& pose) {

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

    ldrp::updateWorldPose(_pose, _pose + 3);     // << timestamp!?

}



static std::mutex _static_alloc_access{};
static std::vector<uint8_t> _temp_bytes{};
static uint8_t* _grid_alloc(size_t s) {
    _temp_bytes.resize(s);
    return _temp_bytes.data();
}

void ros_bridge::collect_map(custom_types::msg::Map& map) {

    ldrp::ObstacleGrid _grid{};

    _static_alloc_access.lock();
    ldrp::getObstacleGrid(_grid, _grid_alloc);
    std::swap(map.map, _temp_bytes);
    _static_alloc_access.unlock();

    map.origin_x_m = _grid.origin_x_m;
    map.origin_y_m = _grid.origin_y_m;
    map.cells_x = _grid.cells_x;
    map.cells_y = _grid.cells_y;
    map.cell_resolution_m = _grid.cell_resolution_m;

}
void ros_bridge::wait_next_map(custom_types::msg::Map& map, double timeout_ms) {

    ldrp::ObstacleGrid _grid{};

    _static_alloc_access.lock();
    ldrp::waitNextObstacleGrid(_grid, _grid_alloc, timeout_ms);
    std::swap(map.map, _temp_bytes);
    _static_alloc_access.unlock();

    map.origin_x_m = _grid.origin_x_m;
    map.origin_y_m = _grid.origin_y_m;
    map.cells_x = _grid.cells_x;
    map.cells_y = _grid.cells_y;
    map.cell_resolution_m = _grid.cell_resolution_m;

}
