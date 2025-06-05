#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <algorithm>
#include <iostream>
#include <vector>

int mode;
std::vector<int> vector;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>> publisher;

std::vector<int> calculate_mode(const std::vector<int>& nums) {
    if (nums.empty()) {
        return {};
    }
    if (nums.size() == 1) {
        return nums;
    }

    std::vector<int> sorted_nums = nums;
    std::sort(sorted_nums.begin(), sorted_nums.end()); 
    int size = sorted_nums.size();
    std::vector<int> modes;
    int maxFrequency = 1, currentFrequency = 1;

    modes.push_back(sorted_nums[0]); // Consider the first element

    for (int i = 1; i < size; i++) {
        if (sorted_nums[i] == sorted_nums[i - 1]) {
            currentFrequency++;
        } else {
            currentFrequency = 1;
        }

        if (currentFrequency > maxFrequency) {
            maxFrequency = currentFrequency;
            modes.clear();
            modes.push_back(sorted_nums[i]);
        } else if (currentFrequency == maxFrequency) {
            if (std::find(modes.begin(), modes.end(), sorted_nums[i]) == modes.end()) {
                modes.push_back(sorted_nums[i]);
            }
        }
    }

    return modes;
}


void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    vector.push_back(msg->data);
    std_msgs::msg::Int32MultiArray out_msg;
    std::cout << "Publicando vector: ";
    for (int num : vector) {
        std::cout << num << " ";
    }
    std::vector<int> modes = calculate_mode(vector);
    std::cout << "Modes: ";
    for (int mode : modes) {
        std::cout << mode << " ";
    }
    out_msg.data = modes;
    std::cout << std::endl;
    publisher->publish(out_msg);
}

int main(int argc, char *argv[]) {
    mode = 0;
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mode");
    auto subscription =
        node->create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);
    
    publisher = node->create_publisher<std_msgs::msg::Int32MultiArray>("mode", 10);

    rclcpp::WallRate loop_rate(2);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}