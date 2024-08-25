#ifndef BAG_LAUNCHER_H
#define BAG_LAUNCHER_H

#include "heartbeat.h"
#include "std_msgs/msg/string.hpp"
#include "bag_recorder/bag_recorder.h"
// #include "bag_launcher.h"
#include <string>
#include <vector>
#include <set>
#include <map>
#include <memory>
#include <bag_recorder/msg/rosbag.hpp>
#include <filesystem>
#include <fstream>
// #include "rclcpp/rclcpp.hpp"

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
            // BagLauncher(const BLOptions bl_options);
            explicit BagLauncher(const BLOptions& options);

            ~BagLauncher();

            void check_all();

        private:
            void Start_Recording(std::shared_ptr<const bag_recorder::msg::Rosbag> msg);
            void Stop_Recording(std::shared_ptr<const std_msgs::msg::String> msg);
            std::string sanitize_topic(std::string topic);
            bool load_config(std::string config_file_name, std::vector<std::string>& topics, std::set<std::string> loaded = {});

        private:
            std::string config_location_;
            std::string data_folder_;
            rclcpp::Subscription<bag_recorder::msg::Rosbag>::SharedPtr record_start_subscriber_;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr record_stop_subscriber_;
            bool publish_name_;
            bool publish_heartbeat_;
            std::string heartbeat_topic_;
            double heartbeat_interval_;
            rclcpp::Publisher<bag_recorder::msg::Rosbag>::SharedPtr name_publisher_;
            bool default_record_all_;
            std::size_t queue_size_;
            rclcpp::TimerBase::SharedPtr heartbeat_timer_;

            std::map<std::string, std::shared_ptr<HeartBeat>> heartbeats_;
            std::map<std::string, std::shared_ptr<BagRecorder>> recorders_;
    };

}
#endif // BAG_LAUNCHER_H