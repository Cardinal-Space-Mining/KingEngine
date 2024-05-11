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

PathPlanNode::PathPlanNode() : Node("path_plan"),
							   node_init(this->config_node()),
							   lidar_data_sub(this->create_subscription<nav_msgs::msg::OccupancyGrid>("obstacle_grid", 1, std::bind(&PathPlanNode::lidar_change_cb, this, std::placeholders::_1))),
							   dest_sub(this->create_subscription<geometry_msgs::msg::PoseStamped>("target_pose", 1, std::bind(&PathPlanNode::destination_change_cb, this, std::placeholders::_1))),
							   location_sub(this->create_subscription<geometry_msgs::msg::PoseStamped>("current_pose", 1, std::bind(&PathPlanNode::location_change_cb, this, std::placeholders::_1))),
							   avoidance_zone_flag_sub(this->create_subscription<std_msgs::msg::Bool>("avoid_zone", 1, std::bind(&PathPlanNode::avoid_zone_flag_change_cb, this, std::placeholders::_1))),
							   path_pub(this->create_publisher<nav_msgs::msg::Path>("/pathplan/nav_path", 1)),
							   weight_map_pub(this->create_publisher<nav_msgs::msg::OccupancyGrid>("/pathplan/nav_map", 1)),
							   robot_width(this->get_parameter(ROBOT_WIDTH_PARAM_NAME).as_double()),
							   turn_cost(this->get_parameter(TURN_COST_PARAM_NAME).as_int()),
							   min_weight(this->get_parameter(MIN_WEIGHT_PARAM_NAME).as_int()),
							   avoidance_barrier_x(this->get_parameter(AVOID_ZONE_X_PARAM_NAME).as_double()),
							   avoidance_barrier_y(this->get_parameter(AVOID_ZONE_Y_PARAM_NAME).as_double()),
							   avoidance_barrier_thickness(this->get_parameter(AVOID_ZONE_THICKNESS_PARAM_NAME).as_double()),
							   output_frame_id(this->get_parameter(OUTPUT_FRAME_PARAM_NAME).as_string()),
							   periodic_publisher(this->create_wall_timer(this->get_parameter(UPDATE_TIME_PARAM_NAME).as_double() * 1000ms, std::bind(&PathPlanNode::export_data, this)))

{
	RCLCPP_INFO(this->get_logger(), "PathPlan Node Initialization!");

#define PRINT_PARAMS_ON_STARTUP
#ifdef PRINT_PARAMS_ON_STARTUP
    RCLCPP_INFO(this->get_logger(), "CONFIG PARAMETERS:\n"
                                    "robot_width: %f\nturn_cost: %d\nmin_weight: %d\n"
                                    "avoidance zones:\n\tcorner_x: %f\n\tcorner_y: %f\n\tbarrier_thickness: %d",
                                    robot_width, turn_cost, min_weight,
                                    avoidance_barrier_x, avoidance_barrier_y,
									avoidance_barrier_thickness);
#endif

	// this->target_pose.position.x = 11.5;
	// this->target_pose.position.y = -4.2;
}

void PathPlanNode::lidar_change_cb(const nav_msgs::msg::OccupancyGrid &map)
{
	// RCLCPP_INFO(this->get_logger(), "Recieved lidar data");

	this->weights = map; // copy occupancy grid directly since we can simply replace our old data

	std::chrono::high_resolution_clock::time_point _t = std::chrono::high_resolution_clock::now();

	const int
		_w = static_cast<int>(this->weights.info.width),
		_h = static_cast<int>(this->weights.info.height);

	cv::Mat _data{_h, _w, CV_8UC1, this->weights.data.data()};
	cv::Mat _dest = cv::Mat::zeros(cv::Size{_w, _h}, CV_8UC1);

    const float
        resolution = this->weights.info.resolution;
	const int
		kernel_diam = static_cast<int>(this->robot_width / resolution) + 1; // actual computation requires half of robot width / res * 2 for diam, but the 0.5 and 2 cancel
	const cv::Size
		kernel_size{kernel_diam, kernel_diam};

	if (include_avoidance_zone) {
		// Draws a vertical line that divides the navigation zone and offload zone
		// defined by the top left corner of offload zone directly downwards to the bottom of the arena
		// int x = (int)(this->avoidance_barrier_x*resolution);
		// int y = (int)(this->avoidance_barrier_y*resolution);
		// while(y < _h) {
		//     _data.at<uint8_t>(x, y) = 255;
		//     y++;
		// }
		cv::line(
			_data,
			cv::Point(this->avoidance_barrier_x*resolution, this->avoidance_barrier_y*resolution), // top left of avoidance zone
			cv::Point(this->avoidance_barrier_x*resolution, _h - 1), // bottom left of avoidance zone
			cv::Scalar(255),
			this->avoidance_barrier_thickness,
			cv::LINE_8);
	}

	cv::dilate(
		_data,
		_dest,
		cv::getStructuringElement(cv::MORPH_ELLIPSE, kernel_size));
	cv::GaussianBlur(
		_dest,
		_data,
		kernel_size,
		10, 10,
		cv::BorderTypes::BORDER_CONSTANT);
	cv::max(
		_data,
		cv::Scalar::all(this->min_weight),
		_data);

	RCLCPP_INFO(this->get_logger(),
				"Weightmap globulization operation completed in %f seconds.",
				std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - _t).count());

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
}

void PathPlanNode::export_data()
{

	if (new_dst || new_map_data)
	{
		nav_msgs::msg::Path ros_path{};

		std::chrono::high_resolution_clock::time_point _t = std::chrono::high_resolution_clock::now();

		std::vector<Eigen::Vector2<int64_t>>
			path = this->nav_map.navigate<uint8_t, false>(
				reinterpret_cast<uint8_t *>(this->weights.data.data()),
				this->weights.info.width,
				this->weights.info.height,
				this->weights.info.origin.position.x,
				this->weights.info.origin.position.y,
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

	if (new_map_data)
	{
		this->weight_map_pub->publish(this->weights);
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
	this->declare_parameter(OUTPUT_FRAME_PARAM_NAME, DEFAULT_OUTPUT_FRAME_ID);
	return true;
}
