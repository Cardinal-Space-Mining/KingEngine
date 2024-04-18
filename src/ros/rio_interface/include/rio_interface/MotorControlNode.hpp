#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "custom_types/msg/motor_neutral.hpp"
#include "custom_types/msg/motor_percent.hpp"
#include "custom_types/msg/motor_velocity.hpp"
#include "SerialMotorCtrl.h"

class MotorCtrlNode : public rclcpp::Node
{
public:
    MotorCtrlNode(MotorSerialConnection &rio_conn);

private:
    void neutral_cb(const custom_types::msg::MotorNeutral& msg);
    void percent_cb(const custom_types::msg::MotorPercent& msg);
    void velocity_cb(const custom_types::msg::MotorVelocity& msg);

    std::mutex locking_mtx;
    MotorSerialConnection rio_conn;

    rclcpp::Subscription<custom_types::msg::MotorNeutral>::SharedPtr neutral_sub;
    rclcpp::Subscription<custom_types::msg::MotorPercent>::SharedPtr percent_sub;
    rclcpp::Subscription<custom_types::msg::MotorVelocity>::SharedPtr velocity_sub;
};