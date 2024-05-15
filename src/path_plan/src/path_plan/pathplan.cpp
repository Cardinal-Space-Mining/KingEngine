/*******************************************************************************
 *   Copyright (C) 2024 Cardinal Space Mining Club                              *
 *                                                                              *
 *   Unless required by applicable law or agreed to in writing, software        *
 *   distributed under the License is distributed on an "AS IS" BASIS,          *
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 *   See the License for the specific language governing permissions and        *
 *   limitations under the License.                                             *
 *                                                                              *
 *                                ;xxxxxxx:                                     *
 *                               ;$$$$$$$$$       ...::..                       *
 *                               $$$$$$$$$$x   .:::::::::::..                   *
 *                            x$$$$$$$$$$$$$$::::::::::::::::.                  *
 *                        :$$$$$&X;      .xX:::::::::::::.::...                 *
 *                .$$Xx++$$$$+  :::.     :;:   .::::::.  ....  :                *
 *               :$$$$$$$$$  ;:      ;xXXXXXXXx  .::.  .::::. .:.               *
 *              :$$$$$$$$: ;      ;xXXXXXXXXXXXXx: ..::::::  .::.               *
 *             ;$$$$$$$$ ::   :;XXXXXXXXXXXXXXXXXX+ .::::.  .:::                *
 *              X$$$$$X : +XXXXXXXXXXXXXXXXXXXXXXXX; .::  .::::.                *
 *               .$$$$ :xXXXXXXXXXXXXXXXXXXXXXXXXXXX.   .:::::.                 *
 *                X$$X XXXXXXXXXXXXXXXXXXXXXXXXXXXXx:  .::::.                   *
 *                $$$:.XXXXXXXXXXXXXXXXXXXXXXXXXXX  ;; ..:.                     *
 *                $$& :XXXXXXXXXXXXXXXXXXXXXXXX;  +XX; X$$;                     *
 *                $$$::XXXXXXXXXXXXXXXXXXXXXX: :XXXXX; X$$;                     *
 *                X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                      *
 *                $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                     *
 *              x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                    *
 *             +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                   *
 *              +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                    *
 *               :$$$$$$$$$. +XXXXXXXXX:      ;: x$$$$$$$$$                     *
 *               ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X                      *
 *              ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.                             *
 *              ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                                *
 *              :;;;;;;;;;;;;.  :$$$$$$$$$$X                                    *
 *               .;;;;;;;;:;;    +$$$$$$$$$                                     *
 *                 .;;;;;;.       X$$$$$$$:                                     *
 *                                                                              *
 *******************************************************************************/

#include "pathplan.hpp"

#include <chrono>
#include <functional>
#include <optional>
#include <utility>
#include <vector>
#include <chrono>

#include <Eigen/Core>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#define avoidance_zone_p1 cv::Point()



using namespace std::literals::chrono_literals;

const std::string PathPlanNode::DEFAULT_OUTPUT_FRAME_ID = "world";

const std::string PathPlanNode::ROBOT_WIDTH_PARAM_NAME = "robot_width";
const std::string PathPlanNode::TURN_COST_PARAM_NAME = "turn_cost";
const std::string PathPlanNode::MIN_WEIGHT_PARAM_NAME = "min_weight";
const std::string PathPlanNode::UPDATE_TIME_PARAM_NAME = "update_time_s";
const std::string PathPlanNode::OUTPUT_FRAME_PARAM_NAME = "output_frame";
const std::string PathPlanNode::AVOID_ZONE_X_PARAM_NAME = "avoidance_zone_corner_x";
const std::string PathPlanNode::AVOID_ZONE_Y_PARAM_NAME = "avoidance_zone_corner_y";
const std::string PathPlanNode::AVOID_ZONE_THICKNESS_PARAM_NAME = "avoidance_zone_barrier_thickness";
const std::string PathPlanNode::ARENA_WIDTH_PARAM_NAME = "arena_width";
const std::string PathPlanNode::ARENA_HEIGHT_PARAM_NAME = "arena_height";

const double ARENA_MAP_PADDING = 0.25;

PathPlanNode::PathPlanNode() : Node("path_plan"),
							   node_init(this->config_node()),
							   lidar_data_sub(this->create_subscription<nav_msgs::msg::OccupancyGrid>("/ldrp/obstacle_grid", 1, std::bind(&PathPlanNode::lidar_change_cb, this, std::placeholders::_1))),
							   dest_sub(this->create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose", 1, std::bind(&PathPlanNode::destination_change_cb, this, std::placeholders::_1))),
							   location_sub(this->create_subscription<geometry_msgs::msg::PoseStamped>("/current_pose", 1, std::bind(&PathPlanNode::location_change_cb, this, std::placeholders::_1))),
							   avoidance_zone_flag_sub(this->create_subscription<std_msgs::msg::Bool>("avoid_zone", 1, std::bind(&PathPlanNode::avoid_zone_flag_change_cb, this, std::placeholders::_1))),
							   end_proc_sub(this->create_subscription<std_msgs::msg::Bool>("end_process", 1, std::bind(&PathPlanNode::end_process_cb, this, std::placeholders::_1))),
							   path_pub(this->create_publisher<nav_msgs::msg::Path>("/pathplan/nav_path", 1)),
							   weight_map_pub(this->create_publisher<nav_msgs::msg::OccupancyGrid>("/pathplan/nav_map", 1)),
							//    raycast_pub(this->create_publisher<nav_msgs::msg::Path>("/ray_path", 1)),
							   raycast_service(this->create_service<custom_types::srv::GetDistToObs>("get_dist_to_obs", std::bind(&PathPlanNode::export_raycast, this, std::placeholders::_1, std::placeholders::_2))),
                               robot_width(this->get_parameter(ROBOT_WIDTH_PARAM_NAME).as_double()),
							   turn_cost(this->get_parameter(TURN_COST_PARAM_NAME).as_int()),
							   min_weight(this->get_parameter(MIN_WEIGHT_PARAM_NAME).as_int()),
							   avoidance_barrier_x(this->get_parameter(AVOID_ZONE_X_PARAM_NAME).as_double()),
							   avoidance_barrier_y(this->get_parameter(AVOID_ZONE_Y_PARAM_NAME).as_double()),
							   avoidance_barrier_thickness(this->get_parameter(AVOID_ZONE_THICKNESS_PARAM_NAME).as_int()),
                               arena_width(this->get_parameter(ARENA_WIDTH_PARAM_NAME).as_double()),
                               arena_height(this->get_parameter(ARENA_HEIGHT_PARAM_NAME).as_double()),
							   output_frame_id(this->get_parameter(OUTPUT_FRAME_PARAM_NAME).as_string()),
							   periodic_publisher(this->create_wall_timer(this->get_parameter(UPDATE_TIME_PARAM_NAME).as_double() * 1000ms, std::bind(&PathPlanNode::export_data, this)))
							   

{
	RCLCPP_INFO(this->get_logger(), "PathPlan Node Initialization!");

#define PRINT_PARAMS_ON_STARTUP
#ifdef PRINT_PARAMS_ON_STARTUP
    RCLCPP_INFO(this->get_logger(), "CONFIG PARAMETERS:\n"
                                    "robot_width: %f\nturn_cost: %d\nmin_weight: %d\n"
                                    "avoidance zones:\n\tcorner_x: %f\n\tcorner_y: %f\n\tbarrier_thickness: %d\n"
                                    "arena_width: %f\narena_height: %f",
                                    robot_width, turn_cost, min_weight,
                                    avoidance_barrier_x, avoidance_barrier_y,
									avoidance_barrier_thickness,
                                    arena_width, arena_height);
#endif

    // Constant data for the published map
    this->weights.header.frame_id = this->output_frame_id;
    this->weights.info.origin.position.x = -ARENA_MAP_PADDING;
    this->weights.info.origin.position.y = -ARENA_MAP_PADDING;
    this->weights.info.origin.position.z = 0.0;
    this->weights.info.origin.orientation.x = 0.0;
    this->weights.info.origin.orientation.y = 0.0;
    this->weights.info.origin.orientation.z = 0.0;
    this->weights.info.origin.orientation.w = 1.0;

	// this->target_pose.position.x = 11.5;
	// this->target_pose.position.y = -4.2;
}

cv::Mat PathPlanNode::crop_to_arena_size(
        const cv::Mat data,
        const int map_w, const int map_h,
        const int arena_min_x, const int arena_min_y,
        const int arena_w, const int arena_h,
        const int edge_padding)
{
    // Pad the data to guarantee the map contains the entire arena (filling unknown spots with 0)

    int padded_arena_w = arena_w + (edge_padding*2);
    int padded_arena_h = arena_h + (edge_padding*2);

    cv::Mat padded_data = cv::Mat::zeros(cv::Size{map_w + (padded_arena_w*2), map_h + (padded_arena_h*2)}, CV_8UC1);
    cv::Rect centered_rect{padded_arena_w, padded_arena_h, map_w, map_h};
    data.copyTo(padded_data(centered_rect));

    // Crop the padded data down to the rectangle of the arena
    cv::Mat arena_mat = cv::Mat::zeros(cv::Size{padded_arena_w, padded_arena_h}, CV_8UC1);

    // Account for translation from padding with the corner coords
    cv::Rect crop_rect{arena_min_x+(padded_arena_w-edge_padding), arena_min_y+(padded_arena_h-edge_padding), padded_arena_w, padded_arena_h};

    (padded_data(crop_rect)).copyTo(arena_mat);

    return arena_mat;
}

void PathPlanNode::globulize() {
    std::chrono::high_resolution_clock::time_point _t = std::chrono::high_resolution_clock::now();

    const float
        resolution = this->input_weights.info.resolution;
    const int
            map_w = static_cast<int>(input_weights.info.width),
            map_h = static_cast<int>(input_weights.info.height),
            arena_w = static_cast<int>(arena_width/resolution),
            arena_h = static_cast<int>(arena_height/resolution),
            map_offset_x = static_cast<int>(input_weights.info.origin.position.x / resolution),
            map_offset_y = static_cast<int>(input_weights.info.origin.position.y / resolution),
            padding = static_cast<int>(ARENA_MAP_PADDING / resolution),
            padded_arena_w = arena_w + 2*padding,
            padded_arena_h = arena_h + 2*padding;

    cv::Mat _data{map_h, map_w, CV_8UC1, input_weights.data.data()};
    cv::Mat arena_weights =
            this->crop_to_arena_size(_data,
                                     map_w, map_h,
                                     -map_offset_x, -map_offset_y,
                                     arena_w, arena_h,
                                     padding);

    // Fill padding with maximum weight
    // left padding
    cv::rectangle(arena_weights,
                  cv::Point{0,0}, cv::Point{padding, padded_arena_h-1}, cv::Scalar(255), cv::FILLED);
    // bottom padding
    cv::rectangle(arena_weights,
                  cv::Point{0,0}, cv::Point{padded_arena_w-1, padding}, cv::Scalar(255), cv::FILLED);
    // right padding
    cv::rectangle(arena_weights,
                  cv::Point{padded_arena_w-1,0}, cv::Point{padded_arena_w-padding, arena_h-1}, cv::Scalar(255), cv::FILLED);
    // top padding
    cv::rectangle(arena_weights,
                  cv::Point{0,padded_arena_h-1}, cv::Point{padded_arena_w-1, padded_arena_h-padding}, cv::Scalar(255), cv::FILLED);

    if (include_avoidance_zone) {
        cv::line(
                arena_weights,
                cv::Point(this->avoidance_barrier_x/resolution - padding, this->avoidance_barrier_y/resolution - padding), // top left of avoidance zone
                cv::Point(this->avoidance_barrier_x/resolution - padding, 0), // bottom left of avoidance zone
                cv::Scalar(255),
                this->avoidance_barrier_thickness,
                cv::LINE_8);
    }

    const int
            kernel_diam = static_cast<int>(this->robot_width / resolution) + 1; // actual computation requires half of robot width / res * 2 for diam, but the 0.5 and 2 cancel
    const cv::Size
            kernel_size{kernel_diam, kernel_diam};

    cv::Mat _dest = cv::Mat::zeros(cv::Size{padded_arena_w, padded_arena_h}, CV_8UC1);

    cv::dilate(
            arena_weights,
            _dest,
            cv::getStructuringElement(cv::MORPH_ELLIPSE, kernel_size));
    cv::GaussianBlur(
            _dest,
            arena_weights,
            kernel_size,
            10, 10,
            cv::BorderTypes::BORDER_REPLICATE);
    cv::max(
            arena_weights,
            cv::Scalar::all(this->min_weight),
            arena_weights);

    RCLCPP_INFO(this->get_logger(),
                "Weightmap globulization operation completed in %f seconds.",
                std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - _t).count());

    this->weights.header.stamp = this->get_clock()->now();
    this->weights.info.map_load_time = this->get_clock()->now();
    this->weights.info.resolution = input_weights.info.resolution;
    this->weights.info.width = padded_arena_w;
    this->weights.info.height = padded_arena_h;
    this->weights.data.clear();
    this->weights.data.resize(padded_arena_w*padded_arena_h);
    std::copy(arena_weights.begin<uint8_t>(), arena_weights.end<uint8_t>(), this->weights.data.begin());
}

void PathPlanNode::lidar_change_cb(const nav_msgs::msg::OccupancyGrid &_map)
{
    this->input_weights = _map;
    this->new_map_data = true;
}

void PathPlanNode::location_change_cb(const geometry_msgs::msg::PoseStamped &msg)
{
	this->current_pose = msg.pose;
}

void PathPlanNode::destination_change_cb(const geometry_msgs::msg::PoseStamped &msg)
{
	this->target_pose = msg.pose;
	this->new_dst = true;
}

void PathPlanNode::avoid_zone_flag_change_cb(const std_msgs::msg::Bool &flag) {
    this->include_avoidance_zone = flag.data;
    this->new_map_data = true;
}

void PathPlanNode::export_data()
{
    if (new_map_data)
    {
        globulize();
        this->weight_map_pub->publish(this->weights);
    }

	if (new_dst || new_map_data)
	{
		nav_msgs::msg::Path ros_path{};

		std::chrono::high_resolution_clock::time_point _t = std::chrono::high_resolution_clock::now();

		std::vector<Eigen::Vector2<int64_t>>
			path = this->nav_map.navigate<uint8_t, false>(
				reinterpret_cast<uint8_t *>(this->weights.data.data()),
				this->weights.info.width,
				this->weights.info.height,
				this->weights.info.origin.position.x - ARENA_MAP_PADDING,
				this->weights.info.origin.position.y - ARENA_MAP_PADDING,
				this->weights.info.resolution,
				this->current_pose.position.x,
				this->current_pose.position.y,
				this->target_pose.position.x,
				this->target_pose.position.y,
				this->turn_cost);

		ros_path.poses.resize(path.size());
		ros_path.header.stamp = this->get_clock()->now();
		ros_path.header.frame_id = this->output_frame_id;

		for (size_t i = 0; i < path.size(); i++)
		{
			ros_path.poses[i].pose.position.x = path[i].x() * this->weights.info.resolution + this->weights.info.origin.position.x;
			ros_path.poses[i].pose.position.y = path[i].y() * this->weights.info.resolution + this->weights.info.origin.position.y;
			ros_path.poses[i].pose.position.z = 0.0;
			ros_path.poses[i].pose.orientation.w = 1.0;
			ros_path.poses[i].pose.orientation.x = 0.0;
			ros_path.poses[i].pose.orientation.y = 0.0;
			ros_path.poses[i].pose.orientation.z = 0.0;
			ros_path.poses[i].header.stamp = ros_path.header.stamp;
			ros_path.poses[i].header.frame_id = this->output_frame_id;
		}

		if (path.size() > 0)
		{
			RCLCPP_INFO(this->get_logger(),
						"Successfully recalculated path in %f seconds with %ld segments.",
						std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - _t).count(),
						path.size());
		}

		this->path_pub->publish(ros_path);
	}

    new_map_data = false;
    new_dst = false;
}


void PathPlanNode::export_raycast(std::shared_ptr<custom_types::srv::GetDistToObs::Request>, std::shared_ptr<custom_types::srv::GetDistToObs::Response> response){
      geometry_msgs::msg::Quaternion q = current_pose.orientation;

      {
          double qq = sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
          q.x /= qq;
          q.y /= qq;
          q.z /= qq;
          q.w /= qq;
      }

      const double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
      const double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
      const double yaw = std::atan2(siny_cosp, cosy_cosp);

      double distance_to_obstacle = this->nav_map.distance_to_obstacle<uint8_t, false>(
              reinterpret_cast<uint8_t *>(this->weights.data.data()),
              this->weights.info.width,
              this->weights.info.height,
              this->weights.info.origin.position.x,
              this->weights.info.origin.position.y,
              this->weights.info.resolution,
              this->current_pose.position.x,
              this->current_pose.position.y,
              yaw,
              this->min_weight);
      RCLCPP_INFO(this->get_logger(), "Yaw: %f\nDistance to obstacle: %f", yaw, distance_to_obstacle);

			response->return_value = distance_to_obstacle;
}

void PathPlanNode::end_process_cb(const std_msgs::msg::Bool &end){
	if(end.data == true){
		RCLCPP_INFO(this->get_logger(),
			"Path Plan Node Exited Successfully.");
		std::exit(0);
	}
}

bool PathPlanNode::config_node()
{
	this->declare_parameter(ROBOT_WIDTH_PARAM_NAME, DEFAULT_ROBOT_WIDTH);
	this->declare_parameter(TURN_COST_PARAM_NAME, DEFAULT_TURN_COST);
	this->declare_parameter(MIN_WEIGHT_PARAM_NAME, DEFAULT_MIN_WEIGHT);
    this->declare_parameter(UPDATE_TIME_PARAM_NAME, DEFAULT_UPDATE_TIME_S);
    this->declare_parameter(AVOID_ZONE_X_PARAM_NAME, DEFAULT_AVOIDANCE_CORNER_X);
    this->declare_parameter(AVOID_ZONE_Y_PARAM_NAME, DEFAULT_AVOIDANCE_CORNER_Y);
    this->declare_parameter(AVOID_ZONE_THICKNESS_PARAM_NAME, DEFAULT_AVOIDANCE_THICKNESS);
    this->declare_parameter(ARENA_WIDTH_PARAM_NAME, DEFAULT_ARENA_WIDTH);
    this->declare_parameter(ARENA_HEIGHT_PARAM_NAME, DEFAULT_ARENA_HEIGHT);
	this->declare_parameter(OUTPUT_FRAME_PARAM_NAME, DEFAULT_OUTPUT_FRAME_ID);
	return true;
}
