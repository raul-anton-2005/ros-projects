#include <chrono> 
#include <cmath>
#include "rclcpp/rclcpp.hpp" // This is the main header file for the ROS 2 C++ client library.
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

double x_position = 0.0;
double y_position = 0.0;
double angle = 0.0;
double initial_x = 0.0;
double initial_y = 0.0;
double initial_angle = 0.0;
bool first_time = true;

double calculate_distance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

double calculate_angle_difference(double initial_angle, double current_angle) {
    double angle_diff = current_angle - initial_angle;
    // Normalize the angle difference to the range [-pi, pi]
    while (angle_diff > M_PI) {
        angle_diff -= 2 * M_PI;
    }
    while (angle_diff < -M_PI) {
        angle_diff += 2 * M_PI;
    }
    return angle_diff;
}

void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (first_time) {
        initial_x = msg->pose.pose.position.x;
        initial_y = msg->pose.pose.position.y;
        auto orientation = msg->pose.pose.orientation;
        double siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y);
        double cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z);
        angle = std::atan2(siny_cosp, cosy_cosp);
        first_time = false;
    }
    x_position = msg->pose.pose.position.x;
    y_position = msg->pose.pose.position.y;
    auto orientation = msg->pose.pose.orientation;
    double siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y);
    double cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z);
    angle = std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv); // Initialize the ROS 2 client library.
    auto node = rclcpp::Node::make_shared("square"); // Create a ROS 2 node.
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    auto subscription = node->create_subscription<nav_msgs::msg::Odometry>("odom", 10, topic_callback);
    node->declare_parameter("linear_speed", 0.1);
    node->declare_parameter("angular_speed", M_PI/20);
    node->declare_parameter("square_length", 1.0);
    
    geometry_msgs::msg::Twist message;
    rclcpp::WallRate loop_rate(10ms);

    double linear_speed = node->get_parameter("linear_speed").get_parameter_value().get<double>();
    double angular_speed = node->get_parameter("angular_speed").get_parameter_value().get<double>();
    double square_length = node->get_parameter("square_length").get_parameter_value().get<double>();

    
    for (int times = 0; times < 4; times++) { 
        initial_x = x_position;
        initial_y = y_position;
        initial_angle = angle;

        std::cout << "Initial position: x = " << x_position << ", y = " << y_position << std::endl;
        std::cout << "Initial angle: " << angle << " radians" << std::endl;

        double distance = 0.0;

        while (rclcpp::ok() && distance < square_length) {
            distance = calculate_distance(initial_x, initial_y, x_position, y_position);
            message.linear.x = linear_speed;
            publisher->publish(message);
            rclcpp::spin_some(node);
            loop_rate.sleep();
        }
        message.linear.x = 0.0;
        message.angular.z = 0.0;
        publisher->publish(message);
        rclcpp::spin_some(node);

        double angle_difference = 0.0;
        
        while (rclcpp::ok() && angle_difference < M_PI/2) {
            angle_difference = calculate_angle_difference(initial_angle, angle);
            message.angular.z = angular_speed;
            publisher->publish(message);
            rclcpp::spin_some(node);
            loop_rate.sleep();
        }
        std::cout << "Final position: x = " << x_position << ", y = " << y_position << std::endl;
        std::cout << "Final angle: " << angle << " radians" << std::endl;
        std::cout << "Distance from initial position: " << distance << " meters" << std::endl;
        std::cout << "Angle difference from initial position: " << angle_difference << " radians" << std::endl;
        std::cout << "----------------------------------------" << std::endl;

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
