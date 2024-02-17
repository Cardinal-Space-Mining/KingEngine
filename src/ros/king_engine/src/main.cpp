#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "custom_types/msg/location.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class LocationSubscriber : public rclcpp::Node
{
  public:
    LocationSubscriber()
    : Node("king_engine")
    {
      publisher_ = this->create_subscription<custom_types::msg::Location>("location", 10, std::bind(&LocationSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const custom_types::msg::Location& msg)
    {

      RCLCPP_INFO(this->get_logger(), "Recieved location: (%F, %F)", msg.x, msg.y);

    }
    rclcpp::Subscription<custom_types::msg::Location>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocationSubscriber>());
  rclcpp::shutdown();
  return 0;
}