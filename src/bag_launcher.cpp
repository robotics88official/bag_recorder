#include "bag_launcher.h"

namespace bag_launcher_node {

/**
 * @brief BLOptions() initializes a BagLauncher options struct with default values
 */
BLOptions::BLOptions() :
    configuration_directory("/data/"),
    data_directory("/media/data"),
    record_start_topic("/bag/start"),
    record_stop_topic("/bag/stop"),
    publish_name(true),
    name_topic("/bag/name"),
    publish_heartbeat(true),
    heartbeat_topic("/bag/heartbeat"),
    heartbeat_interval(10),
    default_record_all(false)
{}

/**
 * @brief BagLauncher() constructor makes BagLauncher node object
 * @param [in] options options to generate the node from
 * @details Initializes subscribers and variables to run node.
 * Initializes name publisher if publish_name is true.
 */
BagLauncher::BagLauncher(const BLOptions& options)
    : Node("bag_launcher"),
      config_location_(options.configuration_directory),
      data_folder_(options.data_directory),
      publish_name_(options.publish_name),
      publish_heartbeat_(options.publish_heartbeat),
      heartbeat_topic_(options.heartbeat_topic),
      heartbeat_interval_(options.heartbeat_interval),
      default_record_all_(options.default_record_all) {

    // Set up subscribers
    record_start_subscriber_ = this->create_subscription<bag_recorder::msg::Rosbag>(
        options.record_start_topic, 10,
        std::bind(&BagLauncher::Start_Recording, this, std::placeholders::_1));

    record_stop_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        options.record_stop_topic, 10,
        std::bind(&BagLauncher::Stop_Recording, this, std::placeholders::_1));

    if (options.publish_name) {
        name_publisher_ = this->create_publisher<bag_recorder::msg::Rosbag>(
            sanitize_topic(options.name_topic), 5);
    }

    // if (options.publish_heartbeat) {
    //     heartbeat_timer_ = this->create_wall_timer(
    //         std::chrono::seconds(heartbeat_interval_),
    //         [this]() { this->publish_heartbeat(); });
    // }
}

/**
 * @brief ~BagLauncher() destructs the bag launcher object safely
 * @details stops each recorder immediately
 */
BagLauncher::~BagLauncher() {
    for (auto& recorder_pair : recorders_) {
        recorder_pair.second->immediate_stop_recording();
    }
}

/**
 * @brief Start_Recording() starts a bag of a certain config type and a given name
 * @param msg ROS msg containing config type and bag name
 * @details creates recorder of given type if it does not exist in map already.
 * Starts the recorder recording. Generates heartbeat publisher if enabled,
 * publishes bag name if enabled.
 */
void BagLauncher::Start_Recording(std::shared_ptr<const bag_recorder::msg::Rosbag> msg) {
    // Find the recorder under that name if it exists already
    auto recorder = recorders_.find(msg->config);
    data_folder_ = msg->data_dir;
    
    // Check and create the directory if it does not exist
    if (!std::filesystem::exists(data_folder_)) {
        RCLCPP_INFO(this->get_logger(), "Folder did not exist, creating directory: %s", data_folder_.c_str());
        std::filesystem::create_directories(data_folder_);
    }

    // If it does not exist, make it
    if (recorder == recorders_.end()) {
        rclcpp::NodeOptions options;

        auto new_recorder = std::make_shared<BagRecorder>(data_folder_, options); //! what to do about options?
        // auto new_recorder = std::make_shared<BagRecorder>(data_folder_);
        recorders_[msg->config] = new_recorder;
    }

    // Make sure bag is not active
    if (recorders_[msg->config]->is_active()) {
        RCLCPP_WARN(this->get_logger(), "Bag configuration %s is already recording to %s.", msg->config.c_str(), recorders_[msg->config]->get_bagname().c_str());
        return;
    }

    // Start recording
    std::vector<std::string> topics;
    std::string full_bag_name = "";
    if (load_config(msg->config, topics)) {
        full_bag_name = recorders_[msg->config]->start_recording(msg->bag_name, topics);
    } else {
        RCLCPP_ERROR(this->get_logger(), "No such config: %s, was able to be loaded from. Recorder not started.", msg->config.c_str());
        return;
    }

    // Make sure there were no errors and bag was made
    if (full_bag_name == "") {
        RCLCPP_WARN(this->get_logger(), "Error prevented %s configuration from recording.", msg->config.c_str());
        return;
    }

    // Publish bag name
    if (publish_name_) {
        bag_recorder::msg::Rosbag message;
        message.config = msg->config;
        message.bag_name = full_bag_name;
        name_publisher_->publish(message);
    }

    // Publish heartbeat
    if (publish_heartbeat_) {
        std_msgs::msg::String message;
        message.data = msg->config;

        // See if it exists already
        auto heartbeat = heartbeats_.find(msg->config);

        // If it doesn't make it
        if (heartbeat == heartbeats_.end()) {
            auto beat = std::make_shared<HeartBeat>(shared_from_this(), heartbeat_topic_, message, heartbeat_interval_);
            heartbeats_[msg->config] = beat;
        }

        // Else start it
        heartbeats_[msg->config]->start();
    }

    RCLCPP_INFO(this->get_logger(), "Recording %s configuration to %s.", msg->config.c_str(), full_bag_name.c_str());
}

/**
* @brief Stop_Recording() stops a bag of a certain configuration
* @param [in] msg ROS string msg containing config type to stop
* @details finds recorder in map by config name, stops it
*/
void BagLauncher::Stop_Recording(std::shared_ptr<const std_msgs::msg::String> msg) {
    // Find recorder in map
    auto recorder = recorders_.find(msg->data);

    // Make sure it exists
    //we do this because a faulty name could be published to us in the message
    //we can't assume we are actually recording msg->data
    if (recorder != recorders_.end()) {
        // Stop the bag
        recorder->second->stop_recording();
        RCLCPP_INFO(this->get_logger(), "%s configuration recorder stopped.", msg->data.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "%s configuration recorder did not exist.", msg->data.c_str());
    }

    if (publish_heartbeat_) {
        // Find heartbeat in map
        auto heartbeat = heartbeats_.find(msg->data);

        // Make sure it exists
        if (heartbeat != heartbeats_.end()) {
            // Stop it
            heartbeat->second->stop();
        }
    }
    return;
} // Stop_Recording()

/**
* @brief sanitize_topic() makes sure a topic is in proper form
* @param [in] topic name of topic to be sanitized
* @return santized topic name
* @details adds '/' to front of string if not there
*/
std::string BagLauncher::sanitize_topic(std::string topic) {
    if (topic.front() != '/') {
        topic = "/" + topic;
    }
    return topic;
} // sanitize_topic()

/**
* @brief load_config() reads in a .config file
* @param [in] config_name the name of the config file to be loaded (without extension)
* @param [in] topics list of topics to push too
* @param [in] loaded set of configs already read from
* @details reads a config file, parses each line into a vector, sanitizes each.
* allows linking to other config files with '$', also allows comments with '#'
* loads other config files with recursion
*/
bool BagLauncher::load_config(std::string config_name, std::vector<std::string>& topics, std::set<std::string> loaded) {
    std::string config_file_name = config_location_ + config_name + ".config";
    std::ifstream fd(config_file_name.c_str());
    std::string line;

    //prevent circular references in config linking
    if (loaded.find(config_name) == loaded.end()) {
        loaded.insert(config_name);
    } else {
        RCLCPP_WARN(this->get_logger(), "%s config loaded already, circular reference detected.", config_name.c_str());
        return false;
    }

    if (!fd) {
        // if this is the first layer by default we record all if no config is
        // found then we can record all
        if (loaded.size() <= 1 && default_record_all_) {
            RCLCPP_ERROR(this->get_logger(), "Topic input file name invalid, recording everything");
            topics.push_back("*");
            return true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Linked config: %s is invalid.", config_name.c_str());
            return false;
        }
    } else {
        while (std::getline(fd, line)) {
            //ignore blank or lines starting with space or #
            if (line.empty() || line.front() == ' ' || line.front() == '#') {
                continue;
            }
            //link to other config files :P
            //interesting but I doubt it's usefullness
            if (line.front() == '$') {
                load_config(line.substr(1), topics, loaded);
                continue;
            }
            topics.push_back(sanitize_topic(line));
        }
        return true;
    }
} // load_config()

/**
* @brief check_all() is called each loop to beat each of the hearts
* @details iterates over all heartbeats and beats each of them
*/
void BagLauncher::check_all() {
    for (auto& heartbeat : heartbeats_) {
        heartbeat.second->beat();
    }
}


} // namespace bag_launcher_node