#include <vector>
#include <optional>
#include <utility>
#include <cstdint>

namespace ros_bridge
{
    using point = std::pair<uint16_t, uint16_t>;
    using path = std::vector<point>;
    using optional_point = std::optional<point>;
    using optional_path = std::optional<path>;

    optional_path on_lidar_data(const std::vector<double>& vec);
    optional_path on_location_change(double x, double y);
    optional_path on_destination_change(double x, double y);
} // namespace ros_bridge


