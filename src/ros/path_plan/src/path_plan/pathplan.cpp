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

#include <Eigen/Core>

// #define HAVE_OPENCV
#ifdef HAVE_OPENCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#endif


PathPlanNode::PathPlanNode(
	float robot_width_m,
	int turn_cost,
	int min_weight
) :
	Node("path_plan"),
	robot_width{robot_width_m},
	turn_cost{turn_cost},
	min_weight{min_weight}
{
	RCLCPP_INFO(this->get_logger(), "PathPlan Node Initialization!");

	// subscribers
	lidar_data_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
		"/ldrp/obstacle_grid", 1,
		std::bind(&PathPlanNode::lidar_change_cb, this, std::placeholders::_1)
	);
	location_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
		"/uesim/pose", 1,
		std::bind(&PathPlanNode::location_change_cb, this, std::placeholders::_1)
	);
	dest_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
		"/destination", 1,
		std::bind(&PathPlanNode::destination_change_cb, this, std::placeholders::_1)
	);

	// publishers
	path_pub = this->create_publisher<nav_msgs::msg::Path>( "/pathplan/nav_path", 1 );
	weight_map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>( "/pathplan/nav_map", 1 );

	this->target_pose.position.x = 11.5;
	this->target_pose.position.y = -4.2;

	// add params? (for turn cost, robot width)
}



void PathPlanNode::lidar_change_cb(const nav_msgs::msg::OccupancyGrid& map) {
	// RCLCPP_INFO(this->get_logger(), "Recieved lidar data");

	this->weights = map;	// copy occupancy grid directly since we can simply replace our old data

#ifdef HAVE_OPENCV

	std::chrono::high_resolution_clock::time_point _t = std::chrono::high_resolution_clock::now();

	const int
		_w = static_cast<int>(this->weights.info.width),
		_h = static_cast<int>(this->weights.info.height);

	cv::Mat _data{ _h, _w, CV_8UC1, this->weights.data.data() };
	cv::Mat _dest = cv::Mat::zeros(cv::Size{ _w, _h }, CV_8UC1);

	const int
		kernel_diam = static_cast<int>(this->robot_width / this->weights.info.resolution) + 1;	// actual computation requires half of robot width / res * 2 for diam, but the 0.5 and 2 cancel
	const cv::Size
		kernel_size{ kernel_diam, kernel_diam };

	cv::dilate(
		_data,
		_dest,
		cv::getStructuringElement(cv::MORPH_ELLIPSE, kernel_size)
	);
	cv::GaussianBlur(
		_dest,
		_data,
		kernel_size,
		10, 10,
		cv::BorderTypes::BORDER_CONSTANT
	);
	cv::max(
		_data,
		cv::Scalar::all(this->min_weight),
		_data
	);

	RCLCPP_INFO(this->get_logger(),
		"Weightmap globulization operation completed in %f seconds.",
		std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - _t).count()
	);

#else

	RCLCPP_INFO(this->get_logger(), "Could not globulize weights since OpenCV is not included.");

#endif

	this->weight_map_pub->publish(this->weights);

	this->recalc_path_and_export();

}

void PathPlanNode::location_change_cb(const geometry_msgs::msg::PoseStamped& msg) {

	this->current_pose = msg.pose;

	// RCLCPP_INFO(this->get_logger(), "Recieved new location: (%F, %F)", this->current_pose.position.x, this->current_pose.position.y);

	// this->recalc_path_and_export();	// don't need to recalc since we are likely following the path (also lidar will be updating this as well)

}

void PathPlanNode::destination_change_cb(const geometry_msgs::msg::PoseStamped& msg) {

	this->target_pose = msg.pose;

	// RCLCPP_INFO(this->get_logger(), "Recieved new destination: (%F, %F)", this->target_pose.position.x, this->target_pose.position.y);

	this->recalc_path_and_export();

}

void PathPlanNode::recalc_path_and_export() {

	nav_msgs::msg::Path ros_path{};

	std::chrono::high_resolution_clock::time_point _t = std::chrono::high_resolution_clock::now();

	std::vector<Eigen::Vector2<int64_t>>
		path = this->nav_map.navigate<uint8_t, false>(
			reinterpret_cast<uint8_t*>(this->weights.data.data()),
			this->weights.info.width,
			this->weights.info.height,
			this->weights.info.origin.position.x,
			this->weights.info.origin.position.y,
			this->weights.info.resolution,
			this->current_pose.position.x,
			this->current_pose.position.y,
			this->target_pose.position.x,
			this->target_pose.position.y,
			this->turn_cost
		);

	ros_path.poses.resize(path.size());
	ros_path.header.stamp = this->get_clock()->now();
	ros_path.header.frame_id = "world";
	for(size_t i = 0; i < path.size(); i++) {
		ros_path.poses[i].pose.position.x = path[i].x() * this->weights.info.resolution + this->weights.info.origin.position.x;
		ros_path.poses[i].pose.position.y = path[i].y() * this->weights.info.resolution + this->weights.info.origin.position.y;
		ros_path.poses[i].pose.position.z = 0.0;
		ros_path.poses[i].pose.orientation.w = 1.0;
		ros_path.poses[i].pose.orientation.x = 0.0;
		ros_path.poses[i].pose.orientation.y = 0.0;
		ros_path.poses[i].pose.orientation.z = 0.0;
		ros_path.poses[i].header.stamp = ros_path.header.stamp;
		ros_path.poses[i].header.frame_id = "world";
	}

	if(path.size() > 0) {
		RCLCPP_INFO(this->get_logger(),
			"Successfully recalculated path in %f seconds with %ld segments.",
			std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - _t).count(),
			path.size()
		);
	}

	this->path_pub->publish(ros_path);

}
