#include <chrono> 
#include <cmath>
#include <iostream>
#include <vector>
#include "rclcpp/rclcpp.hpp" // This is the main header file for the ROS 2 C++ client library.
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> publisher;
float min_range_0_9 = INFINITY;
float min_range_350_359 = INFINITY;
float min_range = INFINITY;
float linear_x = 0.1;
float linear_z = 0.0;
bool turn_left = false;
bool turn_right = false;

void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    std::vector<float> ranges_values_0_9;
    std::vector<float> ranges_values_350_359;
    for (int i = 0; i < 10; i++) {
        if (msg->ranges[i] != 0) {
            ranges_values_0_9.push_back(msg->ranges[i]);
        } else {
            ranges_values_0_9.push_back(INFINITY);
        }
    }   
    for (int i = 350; i < 360; i++) {
        if (msg->ranges[i] != 0) {
            ranges_values_350_359.push_back(msg->ranges[i]);
        } else {
            ranges_values_350_359.push_back(INFINITY);
        }
    }
    
    min_range_0_9 = *std::min_element(ranges_values_0_9.begin(), ranges_values_0_9.end());
    min_range_350_359 = *std::min_element(ranges_values_350_359.begin(), ranges_values_350_359.end());
    min_range = std::min(min_range_0_9, min_range_350_359);
    std::cout << "Min range [0, 9]: " << min_range_0_9 << std::endl;
    std::cout << "Min range [350, 359]: " << min_range_350_359 << std::endl;
    std::cout << "Min range: " << min_range << std::endl;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv); 

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto node = rclcpp::Node::make_shared("wandering"); 
    publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    auto subscription = node->create_subscription<sensor_msgs::msg::LaserScan>("scan", qos, topic_callback);

    rclcpp::WallRate loop_rate(10ms);
    geometry_msgs::msg::Twist message;

    while (rclcpp::ok()) {
        if ((turn_left == false and turn_right == false)) {
            message.linear.x = linear_x;
            message.angular.z = linear_z;
            publisher->publish(message);
            rclcpp::spin_some(node);
            loop_rate.sleep();
            if (min_range <= 1.0) {
                if (min_range_0_9 < min_range_350_359) {
                    turn_right = true;
                } else {
                    turn_left = true;
                }
            }
        } else if (turn_left == true) {
            linear_x = 0.0;
            linear_z = 0.1;
            while (min_range <= 1.0) {
                message.linear.x = linear_x;
                message.angular.z = linear_z;
                publisher->publish(message);
                rclcpp::spin_some(node);
                loop_rate.sleep();
            }
            turn_left = false;
            linear_x = 0.1;
            linear_z = 0.0;
            loop_rate.sleep();
        } else if (turn_right == true) {
            linear_x = 0.0;
            linear_z = -0.1;
            while(min_range <= 1.0) {
                message.linear.x = linear_x;
                message.angular.z = linear_z;
                publisher->publish(message);
                rclcpp::spin_some(node);
                loop_rate.sleep();
            }
            turn_right = false;
            linear_x = 0.1;
            linear_z = 0.0;
            loop_rate.sleep();
        }
    }

    message.linear.x = 0.0;
    message.angular.z = 0.0;
    publisher->publish(message);
    rclcpp::spin_some(node);

    publisher.reset();

    rclcpp::shutdown();
    return 0;
}
