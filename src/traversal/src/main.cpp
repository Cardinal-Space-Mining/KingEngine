#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>
#include <mutex>

#include "traversal/QuaternianHelp.hpp"

#include "rclcpp/rclcpp.hpp"

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "custom_types/srv/set_track_velocity.hpp"
// #include "custom_types/msg/location.hpp"
// #include "custom_types/msg/path.hpp"
// #include "custom_types/msg/map.hpp"
#include "traversal/motion_profile.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

class TraversalNode : public rclcpp::Node
{
public:
  TraversalNode()
      : Node("traversal"),
        motionProfile(std::make_unique<profile>()),
        path_sub(this->create_subscription<nav_msgs::msg::Path>("path", 10, std::bind(&TraversalNode::path_change_cb, this, _1))),
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
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Traversal Node Launched");
  }

  void location_change_cb(const geometry_msgs::msg::PoseStamped &pose)
  {

    RCLCPP_INFO(this->get_logger(), "Recieved location: (%F, %F)", pose.pose.position.x, pose.pose.position.y);
    Quaternion q{pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z};
    auto e = ToEulerAngles(q);
    this->on_location_change(pose.pose.position.x, pose.pose.position.y, e.yaw * (180.0 / 3.141592653589793238463));
  }

  void path_change_cb(const nav_msgs::msg::Path &path)
  {
    RCLCPP_INFO(this->get_logger(), "%s", "Recieved new path");
    std::vector<point> cpath(path.poses.size());
    for (size_t i = 0; i < path.poses.size(); i++)
    {
      cpath[i].x = path.poses[i].pose.position.x;
      cpath[i].y = path.poses[i].pose.position.y;
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
      return result.get()->return_value == 0;
    }
    else
    {
      return false;
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
      return result.get()->return_value == 0;
    }
    else
    {
      return false;
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

    motionProfile->follow_path(); //storing the velocity multipliers  here

    double max_velocity = motionProfile->getMaxVelocity();
    //set the velocities here

    double angular = motionProfile->getAngularVelocity();
            //---------------DIIFF CONST---------------
    // double diffConst = 80;

    // double v1 = motionProfile.getMaxVelocity() - (angular * diffConst);
    // double v2 = motionProfile.getMaxVelocity() + (angular * diffConst);

    double s = motionProfile->getStick();
    double v = 0.25; //Meters per second velocity of the robot
    double theta = abs(angular * 180.0); // go from the angular percentage to degrees. Shouldn't exceed 180, (range is from -180 to 180, but we only want positives here)
    theta = theta * M_PI / 180; //Convert the theta from degrees to radians

    double v1 = ( 2 * v * sin(theta) / s) * ((s / (2 * sin(theta)))  +  .37465) * 167.78;

    double v2 = ( 2 * v * sin(theta) / s) * ((s / (2 * sin(theta)))  -  .37465) * 167.78;


    if (-.3 < angular && angular < 0.3 && !motionProfile->getAtDestination()) { //If we are within 30% (54ish degrees) of our target, then we should not be doing a point turn, unless we are at our destination

      if (angular >= 0) {
            set_right_track_velo(v1);
            set_left_track_velo(v2);
          } else {
            set_right_track_velo(v2);
            set_left_track_velo(v1);
          }
    } else { // set the velocities such that they are doing a point turn. Should we just gun it?
    //Different than a point turn at the end of a path. Don't need to worry about small angular percentages
        if (angular >=0) {
          set_right_track_velo((max_velocity * angular * -1));
          set_left_track_velo(max_velocity * angular);
        }
        else {
          set_right_track_velo(max_velocity * angular);
          set_left_track_velo(max_velocity * angular * -1);
        }
    }
    
    
  }

private:
  std::unique_ptr<profile> motionProfile;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
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