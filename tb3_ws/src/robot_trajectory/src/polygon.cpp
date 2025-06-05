#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp" 
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                         
    auto node = rclcpp::Node::make_shared("polygon"); 
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    node->declare_parameter("linear_speed", 0.8);
    node->declare_parameter("angular_speed", 0.5);
    node->declare_parameter("segment_size", 0.5);
    node->declare_parameter("number_segments", 4);
    geometry_msgs::msg::Twist message;
    rclcpp::WallRate loop_rate(10ms);

    double linear_speed = node->get_parameter("linear_speed").get_parameter_value().get<double>();
    double angular_speed = node->get_parameter("angular_speed").get_parameter_value().get<double>();
    double segment_size = node->get_parameter("segment_size").get_parameter_value().get<double>();
    int number_segments = node->get_parameter("number_segments").get_parameter_value().get<int>();

    double angle = 2 * M_PI / number_segments;

    for (int times = 0; times < number_segments; times++)
    {
        int i = 0, n = segment_size / (0.01 * linear_speed);
        while (rclcpp::ok() && i < n)
        {
            message.linear.x = linear_speed;
            publisher->publish(message);
            rclcpp::spin_some(node);
            loop_rate.sleep();
            i++;
        }
        message.linear.x = 0.0;
        message.angular.z = 0.0;
        publisher->publish(message);
        rclcpp::spin_some(node);

        i = 0;
        n = angle / (0.01 * angular_speed);

        while (rclcpp::ok() && i < n)
        {
            message.angular.z = angular_speed;
            publisher->publish(message);
            rclcpp::spin_some(node);
            loop_rate.sleep();
            i++;
        }

        message.linear.x = 0.0;
        message.angular.z = 0.0;
        publisher->publish(message);
        rclcpp::spin_some(node);
    }

    message.linear.x = 0.0;
    message.angular.z = 0.0;
    publisher->publish(message);
    rclcpp::spin_some(node);

    rclcpp::shutdown(); 
    return 0;
}