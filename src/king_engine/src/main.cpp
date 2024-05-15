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
#include <std_msgs/msg/bool.hpp>

#include "custom_types/srv/start_mining.hpp"
#include "custom_types/srv/stop_mining.hpp"
#include "custom_types/srv/start_offload.hpp"
#include "custom_types/srv/get_dist_to_obs.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

static constexpr double PI{ atan(1) * 4 };

class BoundingBox {
	public:
	// if corners, mining zone, else berm zone
	BoundingBox(double _x1, double _y1, double _x2, double _y2) {
    tlx = _x1;
    tly = _y1;
    brx = _x2;
    bry = _y2;
	}
	double tlx, tly, brx, bry;
};

// TODO set x and y to actual coordinates
const double berm_x = 5.38;
const double berm_y = 0.6;
// (tlx, tly, brx, bry)
const BoundingBox KSC_ARENA_ZONE(0,5,6.88,0);
const BoundingBox KSC_OBS_ZONE(0,5,3.88,0);
const BoundingBox KSC_EXC_ZONE(3.88,5,6.88,0);
const BoundingBox KSC_CON_ZONE(3.88,2,6.88,0);
// const BoundingBox KSC_LBERM_ZONE(4.38,1.1,6.58,0.2);
// const BoundingBox KSC_SBERM_ZONE(4.48,1.0,6.48,0.3);
const BoundingBox KSC_LBERM_ZONE(berm_x - 1.1, berm_y + 0.45, berm_x + 1.1, berm_y - 0.45);
const BoundingBox KSC_SBERM_ZONE(berm_x - 1, berm_y + 0.35, berm_x + 1, berm_y - 0.35);

const double MINING_TIME = 5.0;
const double SAFE_MINING_DIST = 0.25;

template <typename T>
void ke_wait_for_service(T& client){
	while (!client->wait_for_service(1s)) {
		if (!rclcpp::ok()) {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
		std::exit(EXIT_FAILURE);
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
}


class KingEngineNode : public rclcpp::Node
{
public:
	enum class OpMode {
		FINISHED = 0,
		TRAVERSAL = 1,
		MINING = 2,
		OFFLOAD = 3,
		SEARCH_FOR_GOLD = 4
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
		this->location_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/adjusted_pose", 10, std::bind(&KingEngineNode::location_change_cb, this, _1));
		this->destination_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 10);
		// path_pub = this->create_publisher<nav_msgs::msg::Path>("path", 10);
		this->start_mining_service = this->create_client<custom_types::srv::StartMining>("/start_mining");
		this->stop_mining_service = this->create_client<custom_types::srv::StopMining>("/stop_mining");
        this->start_offload_service = this->create_client<custom_types::srv::StartOffload>("/start_offload");
        this->obstacle_distance_service = this->create_client<custom_types::srv::GetDistToObs>("/obstacle_distance");
		this->end_proc_pub = this->create_publisher<std_msgs::msg::Bool>("end_process", 10);

		ke_wait_for_service<decltype(start_mining_service)>(start_mining_service);
		ke_wait_for_service<decltype(stop_mining_service)>(stop_mining_service);
		ke_wait_for_service<decltype(start_offload_service)>(start_offload_service);

    	// TODO if this is to find the total area there is a variable for that now.
		// this->combine_keypoints(
		// 	get_objectives_from_bounding_box(UCFT_EXC_ZONE, 5, 3, 90, OpMode::MINING),
		// 	get_objectives_from_bounding_box(UCFT_LBERM_ZONE, 5, 3, 90, OpMode::OFFLOAD)
		// );

		this->objectives.emplace_back(ObjectiveNode{6.38,4.5,45,OpMode::SEARCH_FOR_GOLD});
		this->objectives.emplace_back(ObjectiveNode{-1,-1,-1,OpMode::MINING});
		this->objectives.emplace_back(ObjectiveNode{berm_x,berm_y,90,OpMode::TRAVERSAL});
		this->objectives.emplace_back(ObjectiveNode{-1,-1,90,OpMode::OFFLOAD});
		this->objectives.emplace_back(ObjectiveNode{-1,-1,-1,OpMode::FINISHED});
		if(this->objectives.size() > 0 && (std::get<3>(this->objectives[0]) == OpMode::TRAVERSAL || std::get<3>(this->objectives[0]) == OpMode::SEARCH_FOR_GOLD)) {
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
				case OpMode::SEARCH_FOR_GOLD: {
					if ((msg.pose.position.x > (KSC_EXC_ZONE.tlx + 0.6)) && 
					(msg.pose.position.y > (KSC_EXC_ZONE.bry + 0.6))) {
                        auto request = std::make_shared<custom_types::srv::GetDistToObs::Request>();
                        auto response = this->obstacle_distance_service->async_send_request(request);
                        rclcpp::spin_until_future_complete(this->shared_from_this(), response);
                        if (response.get()->return_value >= SAFE_MINING_DIST) {
                            this->objective_idx++;
                            current_objective = std::get<3>(this->objectives[this->objective_idx]);
                            // initializations for the next stage occur after this switch case, so break
                            break;
                        }
					}
                    return;
				}
				case OpMode::MINING: {		// the mining service gets started
                    rclcpp::Time current_time = this->get_clock()->now();
					if((current_time - time_since_last_op).seconds() >= MINING_TIME) {	// check that the robot has mined far enough!?
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
					std_msgs::msg::Bool end_all_processes;

					end_all_processes.data = true;

					this->end_proc_pub->publish(end_all_processes);
					std::exit(0);

					return;
				}
				default: {
					return;
				}
			}
			
			// this block only ever runs if an operation finished and we need to process an initialization for the next stage

            time_since_last_op = this->get_clock()->now();

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
    rclcpp::Client<custom_types::srv::GetDistToObs>::SharedPtr obstacle_distance_service;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr end_proc_pub;

	std::vector<ObjectiveNode> objectives{};
	size_t objective_idx{ 0 };

	double
		mining_min_x,
		mining_max_x,
		mining_min_y,
		mining_max_y;
    rclcpp::Time time_since_last_op;
};


int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<KingEngineNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}