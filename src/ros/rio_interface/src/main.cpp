#include "rclcpp/rclcpp.hpp"
#include "custom_types/srv/set_track_velocity.hpp"
#include "custom_types/srv/start_mining.hpp"
#include "custom_types/srv/stop_mining.hpp"
#include "custom_types/srv/start_offload.hpp"
#include "custom_types/srv/stop_offload.hpp"

#include <memory>

void SetTrackVelo(const std::shared_ptr<custom_types::srv::SetTrackVelocity::Request> request,
                  std::shared_ptr<custom_types::srv::SetTrackVelocity::Response> response)
{
    (void)request;
    (void)response;
    // TODO
}

void startMining(const std::shared_ptr<custom_types::srv::StartMining::Request> request,
                 std::shared_ptr<custom_types::srv::StartMining::Response> response)
{
    (void)request;
    (void)response;
    // TODO
}

void stopMining(const std::shared_ptr<custom_types::srv::StopMining::Request> request,
                std::shared_ptr<custom_types::srv::StopMining::Response> response)
{
    (void)request;
    (void)response;
    // TODO
}

void startOffload(const std::shared_ptr<custom_types::srv::StartOffload::Request> request,
                  std::shared_ptr<custom_types::srv::StartOffload::Response> response)
{
    (void)request;
    (void)response;
    // TODO
}

void stopOffload(const std::shared_ptr<custom_types::srv::StopOffload::Request> request,
                 std::shared_ptr<custom_types::srv::StopOffload::Response> response)
{
    (void)request;
    (void)response;
    // TODO
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("rio_interface");

    rclcpp::Service<custom_types::srv::SetTrackVelocity>::SharedPtr service1 = node->create_service<custom_types::srv::SetTrackVelocity>("set_track_velocity", &SetTrackVelo);

    rclcpp::Service<custom_types::srv::StartMining>::SharedPtr service2 = node->create_service<custom_types::srv::StartMining>("start_mining", &startMining);
    rclcpp::Service<custom_types::srv::StopMining>::SharedPtr service3 = node->create_service<custom_types::srv::StopMining>("stop_mining", &stopMining);
    rclcpp::Service<custom_types::srv::StartOffload>::SharedPtr service4 = node->create_service<custom_types::srv::StartOffload>("start_offload", &startOffload);
    rclcpp::Service<custom_types::srv::StopOffload>::SharedPtr service5 = node->create_service<custom_types::srv::StopOffload>("stop_offload", &stopOffload);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to interface.");

    rclcpp::spin(node);
    rclcpp::shutdown();
}