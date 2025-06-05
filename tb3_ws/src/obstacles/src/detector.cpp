#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "example_interfaces/msg/bool.hpp"
#include <iostream>
#include <cmath>

std::shared_ptr<rclcpp::Publisher<example_interfaces::msg::Bool>> publisher;
double obs_angle_min, obs_angle_max, obs_threshold;

void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { 
    example_interfaces::msg::Bool out_msg;
    out_msg.data = false;
    float angle = msg->angle_min;
    for (auto range: msg->ranges) {
        if (angle > M_PI) {
            angle -= 2*M_PI;
        }
        if ((angle >= obs_angle_min) and (angle <= obs_angle_max)) {
            if (range >0.1 and range <= obs_threshold) {
                out_msg.data = true;
            }
        }
        angle += msg->angle_increment;
    }
    publisher->publish(out_msg);
}   

int main(int argc, char *argv[]) { 
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("detector"); 
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto subscription = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan", qos, callback);
    //auto subscription = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, callback);
    publisher = node->create_publisher<example_interfaces::msg::Bool>("obstacle", 10);
    node->declare_parameter("obs_angle_min", -M_PI/8);
    node->declare_parameter("obs_angle_max", M_PI/8);
    node->declare_parameter("obs_threshold", 1.0);
    obs_angle_min = node->get_parameter("obs_angle_min").get_parameter_value().get<double>();
    obs_angle_max = node->get_parameter("obs_angle_max").get_parameter_value().get<double>();
    obs_threshold = node->get_parameter("obs_threshold").get_parameter_value().get<double>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
