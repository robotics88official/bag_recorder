#include "bag_recorder.h"


namespace bag_recorder {

// OutgoingMessage::OutgoingMessage(const std::string& _topic, std::shared_ptr<const rclcpp::SerializedMessage> _msg, std::shared_ptr<std::unordered_map<std::string, std::string>> _connection_header, rclcpp::Time _time) : //! this is diff from orig implementatiom
OutgoingMessage::OutgoingMessage(const std::string& _topic, std::shared_ptr<const std_msgs::msg::String> _msg, std::shared_ptr<std::unordered_map<std::string, std::string>> _connection_header, rclcpp::Time _time) : //! this is diff from orig implementatiom
    topic(_topic), msg(_msg), connection_header(_connection_header), time(_time)
{
}

/**
* @brief BagRecorder() Constructor
* @param [in] data_folder directory to record bags into
* @param [in] append_date whether or not to append the date to the bag_name
* @details Initializes basic variables defining recorder behavior
*/
BagRecorder::BagRecorder(std::string data_folder, const rclcpp::NodeOptions& options = rclcpp::NodeOptions(), bool append_date) :
    Node("bag_recorder", options), data_folder_(data_folder), append_date_(append_date) {

    auto clock = this->get_clock();

    // Check for a valid time, particularly important if using simulated time
    if (!clock->ros_time_is_active()) {
        RCLCPP_WARN(this->get_logger(), "use_sim_time is set to true and no clock is published. Still waiting for valid time...");
    }

    // Wait for a valid time or until the node shuts down
    while (!clock->ros_time_is_active()) {
        if (!rclcpp::ok()) {
            return;
        }
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    RCLCPP_INFO(this->get_logger(), "Valid time received.");
}

/**
* @brief ~BagRecorder() destroys this BagRecorder object
* @details stops recording immediately if currently recording
*/
BagRecorder::~BagRecorder() {
    if(is_active())
        immediate_stop_recording();
    delete message_queue_;
}

/**
* @brief start_recording() starts the bag recorder
* @param [in] bag_name root name of bag to be recorded.
* @param [in] topics vector of topics to be recorded to.
* @return full name of bag that will be recorded to
* @details locks start/stop mutex, generates full bagname, starts bag, starts write thread
*/
std::string BagRecorder::start_recording(std::string bag_name, std::vector<std::string> topics, bool record_all_topics) {
    std::lock_guard<std::mutex> start_stop_lock(start_stop_mutex_);

    // Will not start new bag if there is an active bag already
    if (bag_active_)
        return "";
    bag_active_ = true;
    stop_signal_ = false;
    clear_queue_signal_ = false;

    // Remove the .bag if it is already on the string
    size_t ind = bag_name.rfind(".bag");
    if (ind != std::string::npos && ind == bag_name.size() - 4) {
      bag_name.erase(ind);
    }

    std::string unique_folder = data_folder_ + "bag_" + get_time_str();
    std::filesystem::create_directory(unique_folder); // Use std::filesystem for file operations

    if (append_date_)
        bag_name += "_" + get_time_str();

    if (bag_name.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Bag Name has length 0. Unable to record.");
        return "";
    }

    bag_name += "_" + std::to_string(split_count_) + ".bag";
    bag_filename_ = unique_folder + "/" + bag_name;

    // message_queue_ = std::make_unique<std::queue<OutgoingMessage>>(); // Use unique_ptr for automatic memory management
    message_queue_ = new std::queue<OutgoingMessage>();

    queue_size_ = 0;

    // Test for asterisk to subscribe to all topics
    if (std::any_of(topics.begin(), topics.end(), [](const std::string& topic) { return topic.find("*") != std::string::npos; })) {
        record_all_topics = true;
    }

    if (record_all_topics) {
        recording_all_topics_ = true;
        subscribe_all();
    } else if (!topics.empty()) {
        // Subscribe to specified topics
        for (const auto& topic : topics) {
            // Prevent multiple subscriptions
            if (subscribed_topics_.find(topic) == subscribed_topics_.end()) {
                try {
                    subscribers_.push_back(generate_subscriber(topic));
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Invalid topic name: %s, no subscriber generated. Exception: %s", topic.c_str(), e.what());
                }
            }
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "No Topics Supplied to be recorded. Aborting bag %s.", bag_name.c_str());
        return "";
    }

    start_time_ = this->now(); // Use the node's clock to get the current time
    start_writing();

    // Start write thread
    record_thread_ = std::thread(&BagRecorder::queue_processor, this);
    queue_condition_.notify_all();

    return bag_name;
} // start_recording()

/**
* @brief stop_recording() stops the bag recorder but allows the queue to be emptied
* @details gets start/stop mutex, sets stop flag, kills subscribers
*/
void BagRecorder::stop_recording() {
    std::lock_guard<std::mutex> start_stop_lock(start_stop_mutex_);

    // If not recording then do nothing
    if (!bag_active_)
        return;

    clear_queue_signal_ = true;

    //note that start_stop_lock is acting as a lock for both subscribers_
    //and also for subscribed_topics_
    for (auto &sub : subscribers_) {
        sub.reset();
    }

    subscribed_topics_.clear();

    RCLCPP_INFO(this->get_logger(), "Stopping BagRecorder, clearing queue.");
} // stop_recording()

void BagRecorder::start_writing() {
    // Open bag_ file for writing
    // This got moved here from queue_processor so all the errors that would
    // cause the bag not to start would be in here.
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_filename_ + ".active";
    storage_options.storage_id = "sqlite3";  // Assuming SQLite3 backend

    const rosbag2_cpp::ConverterOptions converter_options(
      {rmw_get_serialization_format(),
       rmw_get_serialization_format()});

    try {
        bag_->open(storage_options, converter_options);
    } catch (const std::runtime_error &e) {
        RCLCPP_ERROR(this->get_logger(), "Error writing: %s", e.what());
        bag_active_ = false;
    }
}

void BagRecorder::stop_writing() {
    RCLCPP_INFO(this->get_logger(), "Closing %s.", bag_filename_.c_str());

    if (bag_active_) {
        bag_.reset();  // This will destruct the object and close the file
        // Rename the file from .active to the final name
        // std::filesystem::rename(rcpputils::fs::path(bag_filename_ + ".active"), rcpputils::fs::path(bag_filename_));
        std::filesystem::rename(bag_filename_ + ".active", bag_filename_);
    }
}

/**
* @brief immediate_stop_recording() stops the bag immediately
* @details obtains the start/stop mutex to set start/stop veriables. Sets a flag
* such that the queue_processor will stop recording immeidately.
* Also unsubscribes from all topics.
*/
void BagRecorder::immediate_stop_recording() {
    //needs the mutex because the stop signals and subscibers can be accessed
    //by the write thread
    std::lock_guard<std::mutex> start_stop_lock(start_stop_mutex_);

    //if not writing then do nothing.
    if (!bag_active_)
        return;

    stop_signal_ = true;

    for (auto &sub : subscribers_) {
        sub.reset();
    }

    subscribed_topics_.clear();

    RCLCPP_INFO(this->get_logger(), "Stopping BagRecorder immediately.");
} // immediate_stop_recording()


/**
* @brief is_active() tells if a bag is currently being recorded to
* @param [in] <name> <parameter_description>
* @return true if currently recording, false if not
*/
bool BagRecorder::is_active() {
    std::lock_guard<std::mutex> start_stop_lock(start_stop_mutex_);
    return bag_active_;
} // is_active()

/**
* @brief get_bagname() returns the filename of the bag being recorded to
* @return returns bag_filename_
* @details returns the class variable of the bag_filename_ even if not
* currently recording
*/
std::string BagRecorder::get_bagname() {
    std::lock_guard<std::mutex> lock(start_stop_mutex_);
    return bag_filename_;
}

/**
* @brief can_log() checks class variables to make sure it is safe to write to file
* @return true if safe to write, false if not
* @details basically returnes checks_failed_, also issues an warning every
* 5 seconds if checks_failed_ is false
*/
bool BagRecorder::can_log() {
    std::lock_guard<std::mutex> lock(start_stop_mutex_);
    if (!checks_failed_)
        return true;

    //sends warning every 5 seconds if this is called continuously
    rclcpp::Time now = this->get_clock()->now();
    if (now >= warn_next_) {
        warn_next_ += rclcpp::Duration::from_seconds(5.0);
        RCLCPP_WARN(this->get_logger(), "Not logging message because logging disabled. Most likely cause is a full disk.");
    }
    return false;
} // can_log()

/**
* @brief is_subscribed_to() sees if the BagRecorder is subscribed to a topic
* @param [in] topic topic to be checked for
* @return true if subscribed, false if not
* @details sees if topic string is located in the set of subscribed_topics_
*/
bool BagRecorder::is_subscribed_to(std::string topic) {
    //all calls to subscribed_topics_ is already protected by the start/stop
    //mutex so I'll just use that here instead of making another call
    std::lock_guard<std::mutex> lock(start_stop_mutex_);
    return (subscribed_topics_.find(topic) != subscribed_topics_.end());
} // is_subscribed_to()

/**
* @brief generate_subscriber() generates a generic subscriber to any topic.
* @param [in] topic topic that the subscriber will be generated for
* @return returns a boost shared_ptr to a ros subscriber
*/
std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> BagRecorder::generate_subscriber(std::string const& topic) {
// std::shared_ptr<rclcpp::Subscription<rclcpp::SerializedMessage>> BagRecorder::generate_subscriber(std::string const& topic) {
    RCLCPP_INFO(this->get_logger(), "Subscribing to %s", topic.c_str());

    auto callback = [this, topic](const std::shared_ptr<const std_msgs::msg::String>&msg) {
    // auto callback = [this, topic](std::shared_ptr<const rclcpp::SerializedMessage> &msg) {
        this->subscriber_callback(msg, topic);
    };

    // auto sub = this->create_subscription<rclcpp::SerializedMessage>(
    auto sub = this->create_subscription<std_msgs::msg::String>(
        topic, rclcpp::QoS(10).best_effort().durability_volatile(), callback);

    subscribed_topics_.insert(topic);
    return sub;
}


/**
* @brief subscriber_callback() takes information from topics it's subscribed to and adds it to the queue
* @param [in] msg_event generic ROS message class
* @param [in] topic topic name in string form
* @param [in] subscriber pointer to the subscriber, fills template requirements but not used
* @param [count] pointer to an in, fills template requirements but not used
* @details turns a message into and OutgoingMessage and adds it to the queue to be written.
*/
// void BagRecorder::subscriber_callback(const std::shared_ptr<const rclcpp::SerializedMessage>& msg, std::string const& topic) {
void BagRecorder::subscriber_callback(const std::shared_ptr<const std_msgs::msg::String> msg, std::string const& topic) {
    auto rectime = this->now();

    // Generate new outgoing message
    auto connection_header = std::make_shared<std::unordered_map<std::string, std::string>>();
    OutgoingMessage out(topic, msg, connection_header, rectime);

    // Writes message to queue
    {
        std::lock_guard<std::mutex> queue_lock(queue_mutex_);
        message_queue_->push(out);
        queue_size_ += out.msg->data.size();
        // queue_size_ += out.msg->size();

        // Make sure we are not exceeding buffer size
        while (buffer_size_ > 0 && queue_size_ > buffer_size_) {
            OutgoingMessage drop = message_queue_->front();
            message_queue_->pop();
            queue_size_ -= drop.msg->data.size();
            // queue_size_ -= drop.msg->size();

            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "rosbag2 record buffer exceeded. Dropping oldest queued message.");
        }
    }

    // Notify write thread
    queue_condition_.notify_all();
} // subscriber_callback()


/**
* @brief queue_processor() actually opens the bag file, writes and closes the bag file
* @details Primary loop locks queue to pull outgoing messages to write to the bag.
* Also locks start/stop variables to see if it needs to stop. Runs scheduled checks.
*/
void BagRecorder::queue_processor() {
    RCLCPP_INFO(this->get_logger(), "Recording to %s.", bag_filename_.c_str());

    auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

    auto check_disk_next = this->now() + rclcpp::Duration::from_seconds(check_disk_interval_);
    auto subscribe_all_next = this->now() + rclcpp::Duration::from_seconds(subscribe_all_interval_);
    
    check_disk();

    std::unique_lock<std::mutex> queue_lock(queue_mutex_, std::defer_lock);

    while (rclcpp::ok() || !message_queue_->empty()) {
        queue_lock.lock();

        // bool finished = false;

        while (message_queue_->empty()) {
            if (stop_signal_ || clear_queue_signal_) {
                // finished = true;
                queue_lock.unlock();
                break;
            }

            if (!rclcpp::ok()) {
                queue_lock.unlock();
                // finished = true;
                break;
            }

            run_scheduled_checks();

            if (!queue_condition_.wait_for(queue_lock, std::chrono::milliseconds(250), [this] { return !message_queue_->empty(); })) {
                continue;
            }
        }

        if (stop_signal_) {
            break;
        }

        if (!message_queue_->empty()) {
            auto out = message_queue_->front();
            message_queue_->pop();
            queue_size_ -= out.msg->data.size();
            // queue_size_ -= out.msg->size();
            queue_lock.unlock();

            bag_message->topic_name = out.topic;
            // auto mutable_msg = std::make_shared<rclcpp::SerializedMessage>(*out.msg);
            // bag_message->serialized_data = std::make_shared<rcutils_uint8_array_t>(mutable_msg->release_rcl_serialized_message());
            auto serializer = rclcpp::Serialization<std_msgs::msg::String>();
            auto serialized_msg = rclcpp::SerializedMessage();
            serializer.serialize_message(out.msg.get(), &serialized_msg);
            bag_message->serialized_data = std::make_shared<rcutils_uint8_array_t>(std::move(serialized_msg.release_rcl_serialized_message()));

            bag_message->time_stamp = out.time.nanoseconds();


            if (can_log()) {
                // bag_->write(out.topic, out.time, *out.msg, out.connection_header);
                bag_->write(bag_message);
            }
        } else {
            queue_lock.unlock();
        }
    }

    stop_writing();
    while (!message_queue_->empty()) {
        message_queue_->pop();
    }

    bag_active_ = false;
}

bool BagRecorder::check_duration(const rclcpp::Time& t) {
    if (t - start_time_ > rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(split_bag_s_ * 1e9))) {
        while (t - start_time_ > rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(split_bag_s_ * 1e9))) {
            stop_writing();
            split_count_++;
            updateFileName();
            start_time_ += rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(split_bag_s_ * 1e9));
            start_writing();
        }
        return true;
    }
    return false;
}

void BagRecorder::updateFileName() {
    int ind = bag_filename_.rfind("_");
    bag_filename_ = bag_filename_.substr(0, ind) + "_" + std::to_string(split_count_) + ".bag";
}


/**
* @brief run_scheduled_checks() sees if a function is scheduled to run and runs it if so
* @details runs both check_disk() and subscribe_all() depending on if they are configured to run and scheduled to.
*/
void BagRecorder::run_scheduled_checks() {
    rclcpp::Time now = this->now();

    if (now < check_disk_next_) {
        check_disk_next_ += rclcpp::Duration::from_seconds(check_disk_interval_);
        check_disk();
    }

    //if any of the stop signals were recieved will not run subscribe all
    if (now >= subscribe_all_next_) {
        subscribe_all_next_ += rclcpp::Duration::from_seconds(subscribe_all_interval_);

        std::scoped_lock<std::mutex> start_stop_lock(start_stop_mutex_);

        if (!(stop_signal_ || clear_queue_signal_) && recording_all_topics_) {
            subscribe_all();
        }
    }
}

/**
* @brief subscribe_all() subscribes to all topics known to master
* @details gets all known topics from master, subscribes if not already subscribed.
*/
void BagRecorder::subscribe_all() {
    //does not call a mutex because all calls to subscribe_all
    //have the mutex already.
    //gets topic info, subscribes to any not already subscribed to
    // std::vector<rclcpp::TopicInfo> topics;
    auto topics = this->get_topic_names_and_types();
    for (const auto& t : topics) {
            if (subscribed_topics_.find(t.first) == subscribed_topics_.end()) {
                subscribers_.push_back(generate_subscriber(t.first));
            }
        }
}

/**
* @brief check_disk() performs various checks on the disk to make sure it is safe to write
* @details Uses statvfs if BOOST version < 3, else uses boost. Sets class
* variable checks_failed_ to true or false depending on results.
*/
void BagRecorder::check_disk() {
    std::filesystem::path p = std::filesystem::absolute(bag_filename_);
    p = p.parent_path();
    std::filesystem::space_info info;
    try {
        info = std::filesystem::space(p);
    } catch (const std::filesystem::filesystem_error& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to check filesystem stats [%s].", e.what());
        checks_failed_ = true;
        return;
    }

    if (info.available < min_recording_space_) {
        RCLCPP_ERROR(this->get_logger(), "Less than %s of space free on disk with %s. Disabling recording.",
                        min_recording_space_str_.c_str(), bag_filename_.c_str());
        checks_failed_ = true;
        return;
    } else if (info.available < 5 * min_recording_space_) {
        RCLCPP_WARN(this->get_logger(), "Less than 5 x %s of space free on disk with %s.", min_recording_space_str_.c_str(), bag_filename_.c_str());
    } else {
        checks_failed_ = false;
    }
}

/**
* @brief get_time_str() returns a timestamp in string form
* @return timestamp in form YYYY-MM-DD-HH-MM-SS
*/
std::string BagRecorder::get_time_str() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream msg;
    msg << std::put_time(std::localtime(&now_c), "%Y-%m-%d_%H-%M-%S");
    return msg.str();
}
} // bag_recorder
