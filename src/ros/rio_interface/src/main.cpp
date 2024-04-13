#include <cstdio>
#include <mutex>   //std::mutex
#include <utility> //std::pair

#include "SerialMotorCtrl.h"
#include "rio_interface/MotorControlNode.hpp"

constexpr const char *RIO_PORT = "/dev/ttyUSB2";

using std::mutex;
using std::pair;
int main(int argc, char **argv)
{
  MotorSerialConnection rio_con(RIO_PORT);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorCtrlNode>(rio_con));
  rclcpp::shutdown();
  return 0;
}
