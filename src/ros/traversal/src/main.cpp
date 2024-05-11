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

class TraversalNode : public rclcpp::Node
{
public:
  TraversalNode()
      : Node("traversal"),
        motionProfile(std::make_unique<profile>()),
        path_sub(this->create_subscription<custom_types::msg::Path>("/path", 10, std::bind(&TraversalNode::path_change_cb, this, _1))),
        location_sub(this->create_subscription<geometry_msgs::msg::PoseStamped>("/adjusted_pose", 10, std::bind(&TraversalNode::location_change_cb, this, _1))),
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
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Traversal Node Launched");
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
    std::vector<point> cpath(path.path.size());
    for (size_t i = 0; i < path.path.size(); i++)
    {
      cpath[i].x = path.path[i].x;
      cpath[i].y = path.path[i].y;
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

  void on_path_change(std::vector<point> &points)
  {
    //update everything?
    motionProfile->compile_path_linear(points);
  }

  void on_location_change(double x_meters, double y_meters, double yaw_degrees)
  {
    //On the change of the location, we should update the motion profile so its current point and heading matches what we get, then we follow the path
    motionProfile->setCurrent(x_meters, y_meters);
    motionProfile->setCurrentHeading(yaw_degrees);

    motionProfile->follow_path();

    double max_velocity = 750;
    //set the velocities here

    double linear = motionProfile->getLinearVelocity();
    double angular = motionProfile->getAngularVelocity();

    double leftVelocity = 0.0;
    double rightVelocity = 0.0;

    double temp_velocity = max_velocity * linear;
    if (linear == 0.0) { //If our linear velocity is 0, then do a pure point turn
      if (angular < 0) { //counterclockwise 
      //   leftVelocity = max_velocity * angular;
      //   rightVelocity = max_velocity * angular * -1; //Angular is negative, so we want right to be positive
      // } else { //clockwise
        leftVelocity = max_velocity * angular;
        rightVelocity = max_velocity * angular * -1; //Anguilar is positive, we want the right to be negative
      }
    } else { //Or else we are doing a 'regular' turn with some forward throttle. Angular velocity determines which side gets more push
    if (angular < 0) {
      leftVelocity = max_velocity * linear * abs(1 - angular);
      rightVelocity = max_velocity * linear * abs(angular);
    } else {
      leftVelocity = max_velocity * linear * angular;
      rightVelocity = max_velocity * linear * (1 - angular);
    }
     
    }

    set_right_track_velo(rightVelocity);

    set_left_track_velo(leftVelocity);
    
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