#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <algorithm>
#include <iostream>
#include <vector>

int max;
std::vector<int> vector;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32>> publisher;


void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    vector.push_back(msg->data);
    max = *std::max_element(vector.begin(), vector.end());
    std::cout << max << std::endl;
    std_msgs::msg::Int32 message;
    message.data = max;
    publisher->publish(message);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("max");
    auto subscription =
        node->create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);
    publisher = node->create_publisher<std_msgs::msg::Int32>("max", 10);

    rclcpp::WallRate loop_rate(2);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}