#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <thread>

// Helper function to publish messages at a high rate
void publish_messages(const std::string& topic) {
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>(topic, 10);
    ros::Rate rate(1000); // High frequency

    int count = 0;
    while (ros::ok() && count < 1000) {
        std_msgs::String msg;
        msg.data = "Test message " + std::to_string(count);
        pub.publish(msg);
        rate.sleep();
        ++count;
    }
}

TEST(MultithreadingTest, HighLoadHandling) {
    std::thread t1(publish_messages, "/test_topic1");
    std::thread t2(publish_messages, "/test_topic2");
    std::thread t3(publish_messages, "/test_topic3");

    // Allow some time for messages to be published
    t1.join();
    t2.join();
    t3.join();

    // Check here if the callbacks were called appropriately
    // This might involve checking a global counter or similar mechanism
    // For example:
    // EXPECT_EQ(global_message_count, 3000); // Assuming each topic sent 1000 messages
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
