#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "example_interfaces/msg/bool.hpp"
#include <iostream>
#include <cmath>

std::shared_ptr<rclcpp::Publisher<example_interfaces::msg::Bool>> publisher;
double person_angle_min, person_angle_max, person_distance_min, person_distance_max;

void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { 
    example_interfaces::msg::Bool out_msg;
    out_msg.data = false;
    float angle = msg->angle_min;
    for (auto range: msg->ranges) {
        if (angle > M_PI) {
            angle -= 2*M_PI;
        }
        if ((angle >= person_angle_min) and (angle <= person_angle_max)) {
            if (range >= person_distance_min and range <= person_distance_max)
            {
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
    publisher = node->create_publisher<example_interfaces::msg::Bool>("person", 10);
    node->declare_parameter("person_angle_min", -M_PI / 12);
    node->declare_parameter("person_angle_max", M_PI / 12);
    node->declare_parameter("person_distance_min", 0.5);
    node->declare_parameter("person_distance_max", 1.3);
    person_angle_min = node->get_parameter("person_angle_min").get_parameter_value().get<double>();
    person_angle_max = node->get_parameter("person_angle_max").get_parameter_value().get<double>();
    person_distance_min = node->get_parameter("person_distance_min").get_parameter_value().get<double>();
    person_distance_max = node->get_parameter("person_distance_max").get_parameter_value().get<double>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
