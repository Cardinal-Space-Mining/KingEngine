#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>
#include <mutex>

#include "traversal/QuaternianHelp.hpp"

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "custom_types/srv/set_track_velocity.hpp"
#include "custom_types/msg/location.hpp"
#include "custom_types/msg/path.hpp"
#include "custom_types/msg/map.hpp"
#include "traversal/motion_profile.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

struct Point
{
  double x_meters;
  double y_meters;
};

class TraversalNode : public rclcpp::Node
{
public:
  TraversalNode()
      : Node("traversal"),
        motionProfile(std::make_unique<profile>()),
        path_sub(this->create_subscription<custom_types::msg::Path>("path", 10, std::bind(&TraversalNode::path_change_cb, this, _1))),
        location_sub(this->create_subscription<geometry_msgs::msg::PoseStamped>("location", 10, std::bind(&TraversalNode::location_change_cb, this, _1))),
        tracks(this->create_client<custom_types::srv::SetTrackVelocity>("set_track_velocity"))
  {
    while (!tracks->wait_for_service(250ms))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        exit(-1);
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
  }

  void location_change_cb(const geometry_msgs::msg::PoseStamped &pose)
  {

    RCLCPP_INFO(this->get_logger(), "Recieved location: (%F, %F)", pose.pose.position.x, pose.pose.position.y);
    Quaternion q{pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z};
    auto e = ToEulerAngles(q);
    this->on_location_change(pose.pose.position.x, pose.pose.position.y, e.yaw * (180.0 / 3.141592653589793238463));
  }

  void path_change_cb(const custom_types::msg::Path &path)
  {
    RCLCPP_INFO(this->get_logger(), "%s", "Recieved new path");
    std::vector<Point> cpath(path.path.size());
    for (size_t i = 0; i < path.path.size(); i++)
    {
      cpath[i].x_meters = path.path[i].x;
      cpath[i].y_meters = path.path[i].y;
    }
    this->on_path_change(cpath);
  }

  bool set_right_track_velo(double turns_per_second)
  {
    auto request = std::make_shared<custom_types::srv::SetTrackVelocity::Request>();
    request->motor_number = custom_types::srv::SetTrackVelocity::Request::TRACK_RIGHT;
    request->velocity_turns_per_second = turns_per_second;
    auto result = tracks->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      return false;
    }
    else
    {
      return result.get()->return_value == 0;
    }
  }

  bool set_left_track_velo(double turns_per_second)
  {
    auto request = std::make_shared<custom_types::srv::SetTrackVelocity::Request>();
    request->motor_number = custom_types::srv::SetTrackVelocity::Request::TRACK_LEFT;
    request->velocity_turns_per_second = turns_per_second;
    auto result = tracks->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      return false;
    }
    else
    {
      return result.get()->return_value == 0;
    }
  }

  void on_path_change(std::vector<Point> &points)
  {
  }

  void on_location_change(double x_meters, double y_meters, double yaw_degrees)
  {
  }

private:
  std::unique_ptr<profile> motionProfile;
  rclcpp::Subscription<custom_types::msg::Path>::SharedPtr path_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr location_sub;
  rclcpp::Client<custom_types::srv::SetTrackVelocity>::SharedPtr tracks;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TraversalNode>());
  rclcpp::shutdown();
  return 0;
}