#include <chrono>
#include <iostream>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "example_interfaces/msg/bool.hpp"

using namespace std::chrono_literals;

enum State {
    GO_FORWARD = 0,
    GO_LEFT = 1,
    GO_RIGHT = 2,
    STOP = 3
};

State state = STOP;
bool front_detection = false;
bool left_detection = false;
bool right_detection = false;

void callback_front(const example_interfaces::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        front_detection = true;
    } else {
        front_detection = false;
    }
}

void callback_left(const example_interfaces::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        left_detection = true;
    } else {
        left_detection = false;
    }
}

void callback_right(const example_interfaces::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        right_detection = true;
    } else {
        right_detection = false;
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("follower");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    auto subs_front = node->create_subscription<example_interfaces::msg::Bool>("/front/person", 10, callback_front);
    auto subs_left = node->create_subscription<example_interfaces::msg::Bool>("/left/person", 10, callback_left);
    auto subs_right = node->create_subscription<example_interfaces::msg::Bool>("/right/person", 10, callback_right);
    geometry_msgs::msg::Twist message;
    rclcpp::WallRate loop_rate(50ms);

    while (rclcpp::ok()) {
        switch (state) {
        case STOP:
            if (right_detection) {
                state = GO_RIGHT;   
            } else if (left_detection) {
                state = GO_LEFT;
            } else if (front_detection) {
                state = GO_FORWARD; 
            }
            break;
        case GO_LEFT:
            if (!left_detection) {
                state = STOP;   
            } else if (front_detection) {
                state = GO_FORWARD; 
            }
            break;
        case GO_RIGHT:
            if (!right_detection) {
                state = STOP;
            }
            else if (front_detection) {
                state = GO_FORWARD;
            }
            break;
        case GO_FORWARD:
            if (!front_detection) {
                state = STOP;
            }
            break;
        }
            // Ejecutar acciones basadas en el estado
        switch (state) {
        case GO_FORWARD:
            message.linear.x = 0.2;
            message.angular.z = 0.0;
            break;
        case GO_LEFT:
            message.linear.x = 0.0;
            message.angular.z = 0.3;
            break;
        case GO_RIGHT:
            message.linear.x = 0.0;
            message.angular.z = -0.3;
            break;
        case STOP:
            message.linear.x = 0.0;
            message.angular.z = 0.0;
            break;
        }
        
        publisher->publish(message);
        rclcpp::spin_some(node);
        loop_rate.sleep();
        }
    rclcpp::shutdown();
    return 0;
} 