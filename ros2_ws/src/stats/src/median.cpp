#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include <iostream>
#include <vector>
#include <algorithm>

float median;
std::vector<float> vector;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> publisher;


void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    int size = vector.size();
    vector.push_back(msg->data);
    std::sort(vector.begin(), vector.end());
    if (size % 2 == 0) {
        median = (vector[size / 2 - 1] + vector[size / 2]) / 2;
    } else {
        median = vector[size / 2];
    }
    std_msgs::msg::Float32 out_msg;
    out_msg.data = median;
    for (int i = 0; i < size; i++) {
        std::cout << vector[i] << " ";
    }
    std::cout << "Median: " << median << std::endl;
    publisher->publish(out_msg);
}

int main(int argc, char *argv[]) {
    median = 0;
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("median");
    auto subscription =
        node->create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);
    
    publisher = node->create_publisher<std_msgs::msg::Float32>("median", 10);

    rclcpp::WallRate loop_rate(2);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}