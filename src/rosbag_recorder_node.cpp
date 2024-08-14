#include "rclcpp/rclcpp.hpp"
#include "bag_launcher.h"

using namespace bag_launcher_node;

int main(int argc, char** argv) {
    // Initialize ROS 2 node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("rosbag_recorder_node");

    BLOptions options;

    // Load parameter strings with no default value
    node->declare_parameter("configuration_directory");
    if (!node->get_parameter("configuration_directory", options.configuration_directory)) {
        RCLCPP_ERROR(node->get_logger(), "Unable to start Bag Recorder Node. No configuration directory supplied.");
        return 0;
    }

    node->declare_parameter("data_directory");
    if (!node->get_parameter("data_directory", options.data_directory)) {
        RCLCPP_ERROR(node->get_logger(), "Unable to start Bag Recorder Node. No data directory supplied.");
        return 0;
    }

    // Load param strings with default values
    options.record_start_topic = node->declare_parameter("start_bag_topic", "/recorder/start");
    options.record_stop_topic = node->declare_parameter("stop_bag_topic", "/recorder/stop");
    options.name_topic = node->declare_parameter("name_topic", "/recorder/bag_name");
    options.heartbeat_topic = node->declare_parameter("heartbeat_topic", "/recorder/heartbeat");

    // Load bool params
    options.publish_name = node->declare_parameter("publish_name", true);
    options.publish_heartbeat = node->declare_parameter("publish_heartbeat", true);
    options.default_record_all = node->declare_parameter("default_record_all", false);

    // Load double param
    options.heartbeat_interval = node->declare_parameter("heartbeat_interval", 10.0);

    // Sanitize the directories
    if (options.configuration_directory.back() != '/')
        options.configuration_directory += "/";
    if (options.data_directory.back() != '/')
        options.data_directory += "/";

    // Print Configuration
    RCLCPP_INFO(node->get_logger(), "[Bag Recorder] Launching.");
    RCLCPP_INFO(node->get_logger(), "[Bag Recorder] Configurations located in %s", options.configuration_directory.c_str());
    RCLCPP_INFO(node->get_logger(), "[Bag Recorder] Data directory located at %s", options.data_directory.c_str());
    RCLCPP_INFO(node->get_logger(), "[Bag Recorder] Start Recording topic: %s", options.record_start_topic.c_str());
    RCLCPP_INFO(node->get_logger(), "[Bag Recorder] Stop Recording topic: %s", options.record_stop_topic.c_str());
    if (options.publish_name) {
        RCLCPP_INFO(node->get_logger(), "[Bag Recorder] Publishing bag names to %s", options.name_topic.c_str());
    }
    if (options.publish_heartbeat) {
        RCLCPP_INFO(node->get_logger(), "[Bag Recorder] Publishing heartbeat every %.2f seconds to %s", options.heartbeat_interval, options.heartbeat_topic.c_str());
    }

    // Make Bag Launcher node
    BagLauncher bag_launcher(node, options);

    // Configure MultiThreadedExecutor
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::create_default_executor_arguments(), 4, true);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
