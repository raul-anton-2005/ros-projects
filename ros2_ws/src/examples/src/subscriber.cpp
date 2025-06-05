#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>

void topic_callback(const std_msgs::msg::String::SharedPtr msg) { // This is the callback function that will get called when a message is received.
    std::cout << msg->data << std::endl; // Print the message to the terminal.
}

int main(int argc, char *argv[]) { 
    rclcpp::init(argc, argv); // Initialize the ROS 2 system.
    auto node = rclcpp::Node::make_shared("subscriber"); // Create a node called "subscriber".
    auto subscription = node->create_subscription<std_msgs::msg::String>("topic", 10, topic_callback); // Create a subscription to the topic "topic" that will call the callback function "topic_callback" when a message is received.
    rclcpp::spin(node); // Spin the node to receive messages. This is used to keep the node running until it is shutdown. Same as while (rclcpp::ok()) { rclcpp::spin_some(node); }
    rclcpp::shutdown(); // Shutdown the ROS 2 system.
    return 0;
}

