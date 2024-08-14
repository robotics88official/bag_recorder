#include "heartbeat.h"

/**
 * @brief HeartBeat() Constructor initializes class variables
 * @param [in] topic topic to publish heartbeat to
 * @param [in] message string to publish on topic
 * @param [in] interval seconds between each heartbeat
 */
HeartBeat::HeartBeat(const std::string & topic, const std_msgs::msg::String & message, double interval)
: Node("heartbeat_publisher"), message_(message) {
    heartbeat_publisher_ = this->create_publisher<std_msgs::msg::String>(topic, 10);
    interval_ = std::chrono::milliseconds(static_cast<int>(interval * 1000));
    beat_ = false;
}

/**
 * @brief start() makes the HeartBeat beat.
 * @details sets beat_ to true, schedules the next heartbeat
 */
void HeartBeat::start() {
    if (!beat_) {
        beat_ = true;
        timer_ = this->create_wall_timer(
            interval_,
            [this]() { this->beat(); }
        );
    }
}

/**
 * @brief stop() makes the heart stop
 * @details sets beat_ to false
 */
void HeartBeat::stop() {
    if (timer_) {
        timer_->cancel();
    }
    beat_ = false;
}

/**
 * @brief beat() publishes the heartbeat
 * @details beats if beat_ is true and is scheduled.
 */
void HeartBeat::beat() {
    if (beat_) {
        heartbeat_publisher_->publish(message_);
    }
}
