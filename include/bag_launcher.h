/// @file bag_launcher.h
/// @author joshs333@live.com
/// @details defines BagLauncher class and includes all headers needed for functions
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "bag_recorder/bag_recorder.h"
#include "bag_recorder/msg/rosbag.hpp"
#include "bag_launcher.h"
#include "heartbeat.h"
#include <string>
#include <vector>
#include <set>
#include <map>
#include <memory>

namespace bag_launcher_node {

    using namespace bag_recorder;

    struct BLOptions {
        BLOptions();

        std::string configuration_directory;
        std::string data_directory;
        std::string record_start_topic;
        std::string record_stop_topic;
        bool publish_name;
        std::string name_topic;
        bool publish_heartbeat;
        std::string heartbeat_topic;
        double heartbeat_interval;
        bool default_record_all;
    };

    class BagLauncher : public rclcpp::Node {
        public:
            // BagLauncher(const rclcpp::NodeOptions& options, BLOptions bl_options);
            BagLauncher(ros::NodeHandle nh, BLOptions options):

            ~BagLauncher();

            void check_all();

        private:
            // void start_recording(const bag_recorder::Rosbag::SharedPtr msg);
            void start_recording(const std::string& config);
            void stop_recording(const std::string& config);
            std::string sanitize_topic(const std::string& topic);
            bool load_config(const std::string& config_file_name, std::vector<std::string>& topics, std::set<std::string> loaded = {});

        private:
            std::string config_location_;
            std::string data_folder_;
            rclcpp::Subscription<bag_recorder::msg::Rosbag>::SharedPtr record_start_subscriber_;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr record_stop_subscriber_;
            bool publish_name_;
            bool publish_heartbeat_;
            std::string heartbeat_topic_;
            double heartbeat_interval_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr name_publisher_;
            bool default_record_all_;
            std::size_t queue_size_;

            std::map<std::string, std::shared_ptr<HeartBeat>> heartbeats_;
            std::map<std::string, std::shared_ptr<BagRecorder>> recorders_;
    };

}