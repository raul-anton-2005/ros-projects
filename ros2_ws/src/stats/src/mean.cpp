#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include <iostream>
#include <queue>

float sum;
int count;
float mean;
float mean_queue;
std::queue<int> queue_sum;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> publisher;

float sum_queue(std::queue<int> q) {
    int sum = 0;
    while (!q.empty()) {
        sum += q.front();  
        q.pop();           
    }
    return sum;
}

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    sum += msg->data;
    int length = queue_sum.size();
    queue_sum.push(sum);
    if (length == 10) {
        mean_queue = sum_queue(queue_sum) / 10;
    } else if (length > 10) {
        queue_sum.pop();
        mean_queue = sum_queue(queue_sum) / 10;
    }
    count++;
    std_msgs::msg::Float32 out_msg;
    mean = sum / count;
    out_msg.data = mean;
    std::cout << "Sum: " << sum << " Count: " << count << " Mean: " << mean << " Mean Queue: " << mean_queue << std::endl;
    publisher->publish(out_msg);
}

int main(int argc, char *argv[]) {
    sum = 0;
    count = 0;
    mean = 0;
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mean");
    auto subscription =
        node->create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);
    
    publisher = node->create_publisher<std_msgs::msg::Float32>("mean", 10);

    rclcpp::WallRate loop_rate(2);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}