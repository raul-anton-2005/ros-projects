#include <chrono>
#include <cmath>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("spiral");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    node->declare_parameter("angular_speed", 0.5);
    node->declare_parameter("distance_between_loops", 1.0);  
    node->declare_parameter("number_of_loops", 3);
    geometry_msgs::msg::Twist message;
    rclcpp::WallRate loop_rate(500ms);

    double angular_speed = node->get_parameter("angular_speed").get_parameter_value().get<double>();
    double distance_between_loops = node->get_parameter("distance_between_loops").get_parameter_value().get<double>();
    int number_of_loops = node->get_parameter("number_of_loops").get_parameter_value().get<int>();

    double loop_time = (2 * M_PI) / angular_speed;  // Tiempo para una vuelta completa
    double dt = 0.5;                                // 0.5s por iteración (500ms)
    int iterations_per_loop = loop_time / dt;
    int total_iterations = (number_of_loops+1) * iterations_per_loop; // +1 para completar la última vuelta

    for (int i = 0; i < total_iterations; i++) {
        double angle = i * dt * angular_speed;
        double linear_speed = angle * angular_speed * distance_between_loops / (2 * M_PI);
        message.linear.x = linear_speed;
        message.angular.z = angular_speed;
        publisher->publish(message);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    message.linear.x = 0.0;
    message.angular.z = 0.0;
    publisher->publish(message);
    rclcpp::shutdown();
    return 0;
}