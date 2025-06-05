#include <chrono>
#include <iostream>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "example_interfaces/msg/bool.hpp"

using namespace std::chrono_literals;

enum State {
    MOVE_FORWARD = 0,
    TURN_LEFT = 1,
    TURN_RIGHT = 2
};

State state = MOVE_FORWARD;
bool front_obstacle = false;
bool left_obstacle = false;
bool right_obstacle = false;

void callback_front(const example_interfaces::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        front_obstacle = true;
        std::cout << "Front obstacle detected" << std::endl;
    } else {
        front_obstacle = false;
        std::cout << "Front obstacle not detected" << std::endl;
    }
}

void callback_left(const example_interfaces::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        left_obstacle = true;
        std::cout << "Left obstacle detected" << std::endl;
    } else {
        left_obstacle = false;
        std::cout << "Left obstacle not detected" << std::endl;
    }
}

void callback_right(const example_interfaces::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        right_obstacle = true;
        std::cout << "Right obstacle detected" << std::endl;
    }   else {
        right_obstacle = false;
        std::cout << "Right obstacle not detected" << std::endl;
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("avoidance");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    auto subs_front = node->create_subscription<example_interfaces::msg::Bool>("/front/obstacle", 10, callback_front);
    auto subs_left = node->create_subscription<example_interfaces::msg::Bool>("/left/obstacle", 10, callback_left);
    auto subs_right = node->create_subscription<example_interfaces::msg::Bool>("/right/obstacle", 10, callback_right);
    geometry_msgs::msg::Twist message;
    rclcpp::WallRate loop_rate(50ms);

    while (rclcpp::ok()) {
        if (front_obstacle && right_obstacle) {
            state = TURN_LEFT; // state = TURN_RIGHT; if we are following something
        } else if (front_obstacle && left_obstacle) {
            state = TURN_RIGHT; // state = TURN_LEFT; if we are following something
        } else if (front_obstacle) {
            state = (rand() % 2 == 0) ? TURN_LEFT : TURN_RIGHT; // Elegir aleatoriamente | state = MOVE_FORWARD; if we are following something
        } else {
            state = MOVE_FORWARD; 
        }

        // Ejecutar acciones basadas en el estado
        switch (state) {
            case MOVE_FORWARD:
                message.linear.x = 0.2;
                message.angular.z = 0.0;
                break;
            case TURN_LEFT:
                message.linear.x = 0.0;
                message.angular.z = 0.2;
                break;
            case TURN_RIGHT:
                message.linear.x = 0.0;
                message.angular.z = -0.2;
                break;
        }
        
        publisher->publish(message);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
} 