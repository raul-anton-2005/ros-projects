#include <chrono> 
#include <cmath>
#include "rclcpp/rclcpp.hpp" // This is the main header file for the ROS 2 C++ client library.
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv); // Initialize the ROS 2 client library.
    auto node = rclcpp::Node::make_shared("square"); // Create a ROS 2 node.
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    geometry_msgs::msg::Twist message;
    rclcpp::WallRate loop_rate(50ms);
    
    double angular_speed = 0.1;
    double linear_speed = 2;

    while (rclcpp::ok()) {
        message.linear.x = linear_speed;
        message.angular.z = angular_speed;

        angular_speed += 0.005;
        if (angular_speed > 3) {
            angular_speed = 3; 
        }

        linear_speed -= 0.0005;

        publisher->publish(message);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    
    message.linear.x = 0.0;
    message.angular.z = 0.0;
    publisher->publish(message);
    rclcpp::shutdown(); // Shutdown the ROS 2 client library.
    return 0;
}
