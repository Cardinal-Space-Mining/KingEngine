#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>
#include <mutex>
#include <queue>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
// #include "custom_types/msg/location.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "custom_types/srv/start_mining.hpp"
#include "custom_types/srv/stop_mining.hpp"
#include "custom_types/srv/start_offload.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

static constexpr double PI{ atan(1) * 4 };

class BoundingBox {
	public:
	// if corners, mining zone, else berm zone
	BoundingBox(double _x1, double _y1, double _x2, double _y2) {
		// if (corners) {
    tlx = _x1;
    tly = _y1;
    brx = _x2;
    bry = _y2;
		// } else {
    // tlx = _x1 - _x2/2.0;
    // tly = _y1 - _x2/2.0;
    // brx = _x1 + _y2/2.0;
    // bry = _y1 + _x2/2.0;
		// }
	}
	double tlx, tly, brx, bry;
};
// tlxtlybry byryx
// bool mining = true;

// BoundingBox UCF_MINING_ZONE(tlx, tly, brx, bry, true);
// BoundingBox UCF_BERM_ZONE(cntrx, cntry, width, height, false);
// const BoundingBox KSC_MINING_ZONE(3.88, 3.5, 6.88, 3.5, true);
// TODO update actual x and y coord of berm zone
// const BoundingBox KSC_BERM_ZONE(5.8, 1.2, 2, 0.7, false);


// (tlx, tly, brx, bry)
const BoundingBox UCFT_ARENA_ZONE(0,8.14,4.57,0);
const BoundingBox UCFT_OBS_ZONE(0,4.07,4.57,0);
const BoundingBox UCFT_EXC_ZONE(0,8.14,4.57,4.07);
const BoundingBox UCFT_CON_ZONE(2.57,8.14,4.57,5.54);
const BoundingBox UCFT_LBERM_ZONE(3.12,7.59,4.02,6.09);
const BoundingBox UCFT_SBERM_ZONE(3.22,7.49,3.92,6.19);

const BoundingBox UCFB_ARENA_ZONE(0,4.57,8.14,0);
const BoundingBox UCFB_OBS_ZONE(0,4.57,4.07,0);
const BoundingBox UCFB_EXC_ZONE(4.07,4.57,8.14,0);
const BoundingBox UCFB_CON_ZONE(5.54,4.57,8.14,2.57);
const BoundingBox UCFB_LBERM_ZONE(6.09,4.02,7.59,3.12);
const BoundingBox UCFB_SBERM_ZONE(6.19,3.92,7.49,3.22);

const BoundingBox KSC_ARENA_ZONE(0,5,6.88,0);
const BoundingBox KSC_OBS_ZONE(0,5,3.88,0);
const BoundingBox KSC_EXC_ZONE(3.88,5,6.88,0);
const BoundingBox KSC_CON_ZONE(3.88,2,6.88,0);
const BoundingBox KSC_LBERM_ZONE(4.38,1.1,6.58,0.2);
const BoundingBox KSC_SBERM_ZONE(4.48,1.0,6.48,0.3);


class KingEngineNode : public rclcpp::Node
{
public:
	enum class OpMode {
		FINISHED = 0,
		TRAVERSAL = 1,
		MINING = 2,
		OFFLOAD = 3
	};
	using ObjectiveNode = std::tuple<double, double, double, OpMode>;	// x, y , theta, operation mode

private:
	static bool pose_inrange(const geometry_msgs::msg::Pose& pose, const ObjectiveNode& target, double trav_epsilon_m = 3e-2, double heading_epsilon_deg = 2.0) {
		const geometry_msgs::msg::Quaternion& q = pose.orientation;
		const double
			yaw = atan2(2.0 * (q.w*q.x + q.y*q.z), (q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)) * (180.0 / PI);	// yaw rotation in degrees
		return (
			std::abs(pose.position.x - std::get<0>(target)) < trav_epsilon_m &&
			std::abs(pose.position.y - std::get<1>(target)) < trav_epsilon_m &&
			std::abs(yaw - std::get<2>(target)) < heading_epsilon_deg
		);
	}
	static bool is_mining_finished(const geometry_msgs::msg::Pose& pose, double x_min, double x_max, double y_min, double y_max) {
		const geometry_msgs::msg::Point& pt = pose.position;
		return (
			(pt.x < x_min || pt.x > x_max) ||
			(pt.y < y_min || pt.y > y_max)
		);
	}
	
	static std::vector<ObjectiveNode> get_objectives_from_bounding_box(BoundingBox bb, int x_divisions, int y_divisions, double theta, OpMode op) {
		std::vector<ObjectiveNode> result{};
		double dx = (bb.brx - bb.tlx) / (float)(x_divisions+1);
		double dy = (bb.bry - bb.tly) / (float)(y_divisions+1);
		for (double y = bb.tly - dy/2.0; y > bb.bry; y += dy) {
			for (double x = bb.tlx + dx/2.0; x < bb.brx; x += dx) {
				result.emplace_back(ObjectiveNode{x, y, theta, op});
			}
		}
		return result;
	}

	void combine_keypoints(const std::vector<ObjectiveNode> &mining, const std::vector<ObjectiveNode> &offload) {
		assert(mining.size() == offload.size());
		for (size_t i = 0; i < mining.size(); i++) {
			this->objectives.emplace_back(mining[i]);
			this->objectives.emplace_back(offload[i]);
		}
	}

	void publish_destination() {
		geometry_msgs::msg::PoseStamped target;

		target.header.frame_id = "world";
		target.header.stamp = this->get_clock()->now();
		target.pose.position.x = std::get<0>(this->objectives[this->objective_idx]);
		target.pose.position.y = std::get<1>(this->objectives[this->objective_idx]);
		target.pose.position.z = 0.0;
		target.pose.orientation.w = cos(std::get<2>(this->objectives[this->objective_idx]) * 0.5);
		target.pose.orientation.x = 0.0;
		target.pose.orientation.y = 0.0;
		target.pose.orientation.z = sin(std::get<2>(this->objectives[this->objective_idx]) * 0.5);

		this->destination_pub->publish(target);
	}


public:
	KingEngineNode() : Node("king_engine")
	{
		this->location_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("location", 10, std::bind(&KingEngineNode::location_change_cb, this, _1));
		this->destination_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("destination", 10);
		// path_pub = this->create_publisher<nav_msgs::msg::Path>("path", 10);
		this->start_mining_service = this->create_client<custom_types::srv::StartMining>("");
		this->stop_mining_service = this->create_client<custom_types::srv::StopMining>("");
		this->start_offload_service = this->create_client<custom_types::srv::StartOffload>("");

    // TODO if this is to find the total area there is a variable for that now.
		// this->combine_keypoints(
		// 	get_objectives_from_bounding_box(UCFT_EXC_ZONE, 5, 3, 90, OpMode::MINING),
		// 	get_objectives_from_bounding_box(UCFT_LBERM_ZONE, 5, 3, 90, OpMode::OFFLOAD)
		// );

		if(this->objectives.size() > 0 && std::get<3>(this->objectives[0]) == OpMode::TRAVERSAL) {
			this->publish_destination();
		}
	}
	void location_change_cb(const geometry_msgs::msg::PoseStamped &msg)
	{
		if (this->objective_idx >= this->objectives.size()) {
			// We have finished all scheduled objectives, we don't have to do anything else
			return;
		}

		OpMode current_objective = std::get<3>(this->objectives[this->objective_idx]);
		while(true) {
			switch(current_objective) {
				case OpMode::TRAVERSAL: {
					if(pose_inrange(msg.pose, this->objectives[this->objective_idx] /** add epsilon params here (uses defaults without) */ )) {
						this->objective_idx++;
						current_objective = std::get<3>(this->objectives[this->objective_idx]);
						// initializations for the next stage occur after this switch case, so break
						break;
					} else {
						return;		// exit since we need to keep traversing
					}
				}
				case OpMode::MINING: {		// the mining service gets started
					if(is_mining_finished(msg.pose, this->mining_min_x, this->mining_max_x, this->mining_min_y, this->mining_max_y)) {	// check that the robot has mined far enough!?
						this->objective_idx++;
						current_objective = std::get<3>(this->objectives[this->objective_idx]);
						// handle going out of mining mode
						if(current_objective != OpMode::MINING) {	// exit mining mode
							// stop mining service
							auto request = std::make_shared<custom_types::srv::StopMining::Request>();
							auto response = this->stop_mining_service->async_send_request(request);
							rclcpp::spin_until_future_complete(this->shared_from_this(), response);		// we don't really care about the return value
						}
						// initializations for the next stage occur after this switch case, so break
						break;
					} else {
						return;		// exit since we need to keep mining
					}
				}
				case OpMode::OFFLOAD: {
					// call the offload service
					auto request = std::make_shared<custom_types::srv::StartOffload::Request>();
					auto response = this->start_offload_service->async_send_request(request);
					rclcpp::spin_until_future_complete(this->shared_from_this(), response);		// we don't really care about the return value

					this->objective_idx++;	// on offload finish
					current_objective = std::get<3>(this->objectives[this->objective_idx]);
					break;	// init for next stage
				}
				case OpMode::FINISHED: {
					// send command to disable robot? (or do an emote/hit the griddy)?
					// falls through to return
					return;
				}
				default: {
					return;
				}
			}
			
			// this block only ever runs if an operation finished and we need to process an initialization for the next stage
			switch(current_objective) {
				case OpMode::MINING: {	// handle going into mining mode
					auto request = std::make_shared<custom_types::srv::StartMining::Request>();
					auto response = this->start_mining_service->async_send_request(request);
					rclcpp::spin_until_future_complete(this->shared_from_this(), response);		// we don't really care about the return value
					return;
				}
				case OpMode::TRAVERSAL: {	// send next target
					this->publish_destination();
					return;
				}
				default: {
					continue;
				}
			}
		}

	}

protected:
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr location_sub;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr destination_pub;

	rclcpp::Client<custom_types::srv::StartMining>::SharedPtr start_mining_service;
	rclcpp::Client<custom_types::srv::StopMining>::SharedPtr stop_mining_service;
	rclcpp::Client<custom_types::srv::StartOffload>::SharedPtr start_offload_service;

	std::vector<ObjectiveNode> objectives{};
	size_t objective_idx{ 0 };

	double
		mining_min_x,
		mining_max_x,
		mining_min_y,
		mining_max_y;
};


int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<KingEngineNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}