#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// #include "rcutils/logging.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <queue>
#include <string>
#include <vector>
#include <set>

namespace bag_recorder {

    class OutgoingMessage
    {
        public:
            OutgoingMessage(std::string const& _topic, std::shared_ptr<rclcpp::SerializedMessage> _msg, std::shared_ptr<std::unordered_map<std::string, std::string>> _connection_header, rclcpp::Time _time);

            std::string                                             topic;
            std::shared_ptr<rclcpp::SerializedMessage>              msg;
            std::shared_ptr<std::unordered_map<std::string, std::string>> connection_header;
            rclcpp::Time                                            time;
    };

    class BagRecorder : public rclcpp::Node
    {
        public:
            BagRecorder(std::string data_folder, bool append_date = true);
            ~BagRecorder();

            std::string get_bagname();

            std::string start_recording(std::string bag__name, std::vector<std::string> topics, bool record_all_topics = false); //-- initializes subscribers starts record thread, initializes all record variables, generates bag_ name
            void stop_recording(); //-- kills subscribers, sets flag for write_thread to stop after queue is cleared
            void start_writing();
            void stop_writing();
            void immediate_stop_recording(); //-- kills subscribers, sets flag for write_thread to stop recording immediately
            void updateFileName();
            bool check_duration(const rclcpp::Time& t);

            //status check functions
            bool is_active(); //-- if there is a bag_ being recorded to
            bool can_log(); //-- if the BagRecorder can atually log to the file
            bool is_subscribed_to(std::string topic); //-- if the BagRecorder is currently subscribed to this topic


        private:
            //generates a subcriber
            void subscribe_all();
            rclcpp::SubscriptionBase::SharedPtr generate_subscriber(std::string const& topic);
            void subscriber_callback(const std::shared_ptr<rclcpp::SerializedMessage> msg, std::string const& topic);
            void unsubscribe_all();

            //write thread
            void queue_processor(); //-- starts bag_, writes to queue until end conditin reached, closes bag_

            //helper functions to check for errors
            void run_scheduled_checks(); //-- sees if check_disk is scheduled, run if so and schedule next check
            void check_disk(); //-- checks disk to see if there is space to write or whatnot

            //helper function to generate bag__name
            static std::string get_time_str(); //-- turns  ros time into string


        private:
            std::string                             data_folder_;
            bool                                    append_date_;
            bool                                    recording_all_topics_;
            size_t                                  min_recording_space_ = 1024 * 1024 * 1024;
            std::string                             min_recording_space_str_ = "1G";
            size_t                                  buffer_size_ = 1048576 * 256;

            rosbag2_cpp::writers::SequentialWriter  bag_;
            std::string                             bag_filename_;
            bool                                    bag_active_ = false;

            rclcpp::Time                            start_time_;
            float                                   split_bag_s_ = 60.0;
            int                                     split_count_ = 0;

            bool                                    clear_queue_signal_;
            bool                                    stop_signal_;
            boost::mutex                            start_stop_mutex_;

            std::vector<rclcpp::SubscriptionBase::SharedPtr> subscribers_;
            std::set<std::string>                   subscribed_topics_;
            rclcpp::Time                            subscribe_all_next_;
            boost::mutex                            subscribers_mutex_;

            boost::condition_variable_any           queue_condition_;
            boost::mutex                            queue_mutex_;
            std::queue<OutgoingMessage>*            message_queue_;

            size_t                                  queue_size_ = 0;

            boost::thread                           record_thread_;

            bool                                    checks_failed_ = false;
            rclcpp::Time                            check_disk_next_;
            rclcpp::Time                            warn_next_;

            double                                  subscribe_all_interval_ = 5.0;
            double                                  check_disk_interval_ = 10.0;
    };
}
