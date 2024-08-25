#ifndef HEARTBEAT_H
#define HEARTBEAT_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
#include <chrono>

class HeartBeat : public rclcpp::Node {
public:
    HeartBeat(std::shared_ptr<rclcpp::Node> node, const std::string& topic, const std_msgs::msg::String& message, double interval);
    void start();
    void stop();
    void beat();

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_publisher_;
    std_msgs::msg::String message_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds interval_;
    bool beat_ = false;
};
#endif // HEARTBEAT_H