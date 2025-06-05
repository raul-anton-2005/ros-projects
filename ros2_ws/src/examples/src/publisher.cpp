#include <chrono> 
#include "rclcpp/rclcpp.hpp" // This is the main header file for the ROS 2 C++ client library.
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv); // Initialize the ROS 2 client library.
    auto node = rclcpp::Node::make_shared("publisher"); // Create a ROS 2 node with the name "publisher".
    auto publisher = node->create_publisher<std_msgs::msg::String>("topic", 10); // Create a ROS 2 publisher. If the topic does not exist, it will be created.
    std_msgs::msg::String message; // Create a ROS 2 message.
    auto publish_count = 0; 
    rclcpp::WallRate loop_rate(500ms); // Create a rate object with a frequency of 2 Hz.
    
    while (rclcpp::ok()) { // Check if the ROS 2 client library is still running.
        message.data = "Hello, world! " + std::to_string(publish_count++); // Set the message data.
        publisher->publish(message); // Publish the message.
        rclcpp::spin_some(node); // This function will check for new messages and call the appropriate callbacks giving back control to the executor.
        loop_rate.sleep(); // Sleep for the remainder of the period.
    }    
    rclcpp::shutdown(); // Shutdown the ROS 2 client library.
    return 0;
}
