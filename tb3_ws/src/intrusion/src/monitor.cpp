#include <chrono>
#include <iostream>

#include "example_interfaces/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

enum State {
    STAY = 0,
    TURN_N = 1,
    TURN_NE = 2,
    TURN_EAST = 3,
    TURN_SE = 4,
    TURN_SOUTH = 5,
    TURN_SW = 6,
    TURN_WEST = 7,
    TURN_NW = 8,
};

State current_state = STAY;

bool north = false;
bool northeast = false;
bool east = false;
bool southeast = false;
bool south_1 = false;
bool south_2 = false;
bool south = false;
bool southwest = false;
bool west = false;
bool northwest = false;

double angle = 0.0;
double angle_ini = 0.0;
double ang_nor = 0.0;          // North (0°)
double ang_nore = M_PI / 4;    // Northeast (0.785 rad)
double ang_ea = M_PI / 2;      // East (1.57 rad)
double ang_se = 3 * M_PI / 4;  // Southeast (2.36 rad)
double ang_sou = 3.13;         // South (3.13 rad para evitar problemas de cambio de signo)
double ang_sw = 3 * M_PI / 4;  // Southwest (2.36 rad)
double ang_we = M_PI / 2;      // West (1.57 rad)
double ang_norw = M_PI / 4;    // Northwest (0.785 rad)

double angle_diff = 0.0;

double calculate_angle_diff(double initial_angle, double current_angle) {
    double angle_diff = current_angle - initial_angle;
    // Normalizar la diferencia entre [-pi, pi]
    while (angle_diff > M_PI) {
        angle_diff -= 2 * M_PI;
    }
    while (angle_diff < -M_PI) {
        angle_diff += 2 * M_PI;
    }
    std::cout << "Angle initial: " << initial_angle << std::endl;
    std::cout << "Angle current: " << current_angle << std::endl;
    std::cout << "Angle difference: " << angle_diff << std::endl;
    return fabs(angle_diff);
}

void north_callback(const example_interfaces::msg::Bool::SharedPtr msg) {
    north = msg->data;
}

void northeast_callback(const example_interfaces::msg::Bool::SharedPtr msg) {
    northeast = msg->data;
}

void east_callback(const example_interfaces::msg::Bool::SharedPtr msg) {
    east = msg->data;
}
void southwest_callback(const example_interfaces::msg::Bool::SharedPtr msg) {
    southwest = msg->data;
}

void south_1_callback(const example_interfaces::msg::Bool::SharedPtr msg) {
    south_1 = msg->data;
}

void south_2_callback(const example_interfaces::msg::Bool::SharedPtr msg) {
    south_2 = msg->data;
}

void southeast_callback(const example_interfaces::msg::Bool::SharedPtr msg) {
    southeast = msg->data;
}
void west_callback(const example_interfaces::msg::Bool::SharedPtr msg) {
    west = msg->data;
}

void northwest_callback(const example_interfaces::msg::Bool::SharedPtr msg) {
    northwest = msg->data;
}

void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    auto orientation = msg->pose.pose.orientation;
    double siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y);
    double cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z);
    angle = std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("monitor");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    auto north_sub = node->create_subscription<example_interfaces::msg::Bool>("/north/obstacle", 10, north_callback);
    auto northeast_sub = node->create_subscription<example_interfaces::msg::Bool>("/northeast/obstacle", 10, northeast_callback);
    auto east_sub = node->create_subscription<example_interfaces::msg::Bool>("/east/obstacle", 10, east_callback);
    auto southwest_sub = node->create_subscription<example_interfaces::msg::Bool>("/southwest/obstacle", 10, southwest_callback);
    auto south_sub_1 = node->create_subscription<example_interfaces::msg::Bool>("/south_1/obstacle", 10, south_1_callback);
    auto south_sub_2 = node->create_subscription<example_interfaces::msg::Bool>("/south_2/obstacle", 10, south_2_callback);
    auto southeast_sub = node->create_subscription<example_interfaces::msg::Bool>("/southeast/obstacle", 10, southeast_callback);
    auto west_sub = node->create_subscription<example_interfaces::msg::Bool>("/west/obstacle", 10, west_callback);
    auto northwest_sub = node->create_subscription<example_interfaces::msg::Bool>("/northwest/obstacle", 10, northwest_callback);

    auto odometry = node->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, odom_callback);

    geometry_msgs::msg::Twist message;
    rclcpp::WallRate loop_rate(10ms);
    double angular_speed = 0.3;

    while (rclcpp::ok()) {
        south = south_1 or south_2; // Detectara sur si detecta en uno de los dos rangos de sur
        switch (current_state) {
            case STAY:
                if (north) {  
                    current_state = TURN_N;
                } else if (northeast) {
                    current_state = TURN_NE;
                } else if (east) {
                    current_state = TURN_EAST; 
                } else if (southeast) {
                    current_state = TURN_SE; 
                } else if (south) {
                    current_state = TURN_SOUTH; 
                } else if (southwest) {
                    current_state = TURN_SW; 
                } else if (west) {
                    current_state = TURN_WEST;
                } else if (northwest) {
                    current_state = TURN_NW;
                } else {
                    message.linear.x = 0.0;
                    message.angular.z = 0.0;
                }
                break;
            case TURN_N:
                message.angular.z = 0.0; // Si detecta delante no se mueve
                publisher->publish(message);
                rclcpp::spin_some(node);
                loop_rate.sleep();
                if (!north) {
                    current_state = STAY;
                }
                break;
            case TURN_NE:
                angle_ini = angle;
                angle_diff = 0.0;
                while (angle_diff < ang_nore) {
                    angle_diff = calculate_angle_diff(angle_ini, angle);
                    message.angular.z = -angular_speed;
                    publisher->publish(message);
                    rclcpp::spin_some(node);
                    loop_rate.sleep();
                }
                if (!northeast) {
                    current_state = STAY;
                }
                break;
            case TURN_EAST:
                angle_ini = angle;
                angle_diff = 0.0;
                while (angle_diff < ang_ea) {
                    angle_diff = calculate_angle_diff(angle_ini, angle);
                    message.angular.z = -angular_speed;
                    publisher->publish(message);
                    rclcpp::spin_some(node);
                    loop_rate.sleep();
                }
                if (!east) {
                    current_state = STAY;
                }
                break;
            case TURN_SE:
                angle_ini = angle;
                angle_diff = 0.0;
                while (angle_diff < ang_se) {
                    angle_diff = calculate_angle_diff(angle_ini, angle);
                    message.angular.z = -angular_speed;
                    publisher->publish(message);
                    rclcpp::spin_some(node);
                    loop_rate.sleep();
                }
                if (!southeast) {
                    current_state = STAY;
                }
                break;
            case TURN_SOUTH:
                angle_ini = angle;
                angle_diff = 0.0;
                while (angle_diff < ang_sou) {
                    angle_diff = calculate_angle_diff(angle_ini, angle);
                    message.angular.z = angular_speed;
                    publisher->publish(message);
                    rclcpp::spin_some(node);
                    loop_rate.sleep();
                }
                south = south_1 or south_2;
                if (!south) {
                    current_state = STAY;
                }
                break;
            case TURN_SW:
                angle_ini = angle;
                angle_diff = 0.0;
                while (angle_diff < ang_sw) {
                    angle_diff = calculate_angle_diff(angle_ini, angle);
                    message.angular.z = angular_speed;
                    publisher->publish(message);
                    rclcpp::spin_some(node);
                    loop_rate.sleep();
                }
                if (!southwest) {
                    current_state = STAY;
                }
                break;
            case TURN_WEST:
                angle_ini = angle;
                angle_diff = 0.0;
                while (angle_diff < ang_we) {
                    angle_diff = calculate_angle_diff(angle_ini, angle);
                    message.angular.z = angular_speed;
                    publisher->publish(message);
                    rclcpp::spin_some(node);
                    loop_rate.sleep();
                }
                if (!west) {
                    current_state = STAY;
                }
                break;
            case TURN_NW:
                angle_ini = angle;
                angle_diff = 0.0;
                while (angle_diff < ang_norw) {
                    angle_diff = calculate_angle_diff(angle_ini, angle);
                    message.angular.z = angular_speed;
                    publisher->publish(message);
                    rclcpp::spin_some(node);
                    loop_rate.sleep();
                }
                if (!northwest) {
                    current_state = STAY;
                }
                break;
        }

        publisher->publish(message);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}