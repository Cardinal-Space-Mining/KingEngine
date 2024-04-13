#include "rio_interface/MotorControlNode.hpp"
#include <functional>

using custom_types::msg::MotorNeutral;
using custom_types::msg::MotorPercent;
using custom_types::msg::MotorVelocity;
using std::placeholders::_1;

MotorCtrlNode::MotorCtrlNode(MotorSerialConnection &rio_conn) : Node("motor_connector"), rio_conn(rio_conn)
{
    this->neutral_sub = this->create_subscription<MotorNeutral>("motor_neutral_mode_updates", 10, std::bind(&MotorCtrlNode::neutral_cb, this, _1));
    this->percent_sub = this->create_subscription<MotorPercent>("motor_percent_ctrl_updates", 10, std::bind(&MotorCtrlNode::percent_cb, this, _1));
    this->velocity_sub = this->create_subscription<MotorVelocity>("motor_velocity_ctrl_updates", 10, std::bind(&MotorCtrlNode::velocity_cb, this, _1));
}

void MotorCtrlNode::neutral_cb(const MotorNeutral &msg)
{
    std::lock_guard<std::mutex>(this->locking_mtx);
    if (msg.coast_t_or_break_f == true)
    {
        this->rio_conn.set_neutral_mode(msg.motor_id, MotorNeutralMode::MOTOR_COAST);
    }
    else
    {
        this->rio_conn.set_neutral_mode(msg.motor_id, MotorNeutralMode::MOTOR_BREAK);
    }
}

void MotorCtrlNode::percent_cb(const custom_types::msg::MotorPercent &msg)
{
    std::lock_guard<std::mutex>(this->locking_mtx);
    this->rio_conn.set_percent(msg.motor_id, msg.motor_percent);
}

void MotorCtrlNode::velocity_cb(const custom_types::msg::MotorVelocity &msg)
{
    std::lock_guard<std::mutex>(this->locking_mtx);
    this->rio_conn.set_percent(msg.motor_id, msg.velocity_turns_per_second);
}