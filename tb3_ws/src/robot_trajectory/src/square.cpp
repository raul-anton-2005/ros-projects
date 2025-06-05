#include <chrono> 
#include <cmath>
#include "rclcpp/rclcpp.hpp" // This is the main header file for the ROS 2 C++ client library.
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv); // Initialize the ROS 2 client library.
    auto node = rclcpp::Node::make_shared("square"); // Create a ROS 2 node.
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    node->declare_parameter("linear_speed", 0.1);
    node->declare_parameter("angular_speed", M_PI/20);
    node->declare_parameter("square_length", 1.0);
    geometry_msgs::msg::Twist message;
    rclcpp::WallRate loop_rate(10ms);

    double linear_speed = node->get_parameter("linear_speed").get_parameter_value().get<double>();
    double angular_speed = node->get_parameter("angular_speed").get_parameter_value().get<double>();
    double square_length = node->get_parameter("square_length").get_parameter_value().get<double>();
    for (int times = 0; times < 4; times++) { 
        int i = 0, n = square_length/(0.01 * linear_speed);
        while (rclcpp::ok() && i < n) {
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
        n = M_PI/(0.01965 * angular_speed);

        while (rclcpp::ok() && i < n) {
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

    rclcpp::shutdown(); // Shutdown the ROS 2 client library.
    return 0;
}
