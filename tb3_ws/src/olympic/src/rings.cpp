#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>
#include "rclcpp/rclcpp.hpp"           // This is the main header file for the ROS 2 C++ client library.
#include "geometry_msgs/msg/twist.hpp" // This header file defines the Twist message type.
#include "turtlesim/srv/set_pen.hpp"  // This header file defines the SetPen service type.
#include "turtlesim/srv/teleport_absolute.hpp" // This header file defines the TeleportAbsolute service type.

using namespace std::chrono_literals;

double radius;

void change_pen(const std::shared_ptr<rclcpp::Node> &node, 
    const std::shared_ptr<rclcpp::Client<turtlesim::srv::SetPen>> &client, 
    int r, int g, int b, int width, int off)
{
    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->r = r;
    request->g = g;
    request->b = b;
    request->width = width;
    request->off = off;

    while (!client->wait_for_service(1s)) 
    {
        if (rclcpp::ok())
        {
            RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Set pen color to r:%d, g:%d, b:%d", r, g, b);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service set_pen");
    }
}

void teleport_absolute(const std::shared_ptr<rclcpp::Node> &node, 
    const std::shared_ptr<rclcpp::Client<turtlesim::srv::TeleportAbsolute>> &client, 
    double x, double y, double theta)
{
    auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    request->x = x;
    request->y = y;
    request->theta = theta;

    while (!client->wait_for_service(1s)) 
    {
        if (rclcpp::ok())
        {
            RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Teleported to x:%f, y:%f, theta:%f", x, y, theta);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service teleport_absolute");
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rings");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    auto client_pen = node->create_client<turtlesim::srv::SetPen>("turtle1/set_pen");
    auto client_teleport = node->create_client<turtlesim::srv::TeleportAbsolute>("turtle1/teleport_absolute");
    geometry_msgs::msg::Twist msg;
    rclcpp::WallRate loop_rate(50ms);

    node->declare_parameter("radius", 1.0);
    radius = node->get_parameter("radius").get_parameter_value().get<double>();

    double linear_velocity = 1.0;
    double angular_velocity = linear_velocity / radius; // Calcular la velocidad angular basada en la velocidad lineal y el radio.
    
    double circle_perimeter = 2 * M_PI * radius; // Calcular el perímetro del círculo.
    double sep = radius * 2.4;

    std::vector<std::vector<double>> positions = {
        {2.8, 7.0},
        {2.8 + sep, 7.0},
        {2.8 + 2 * sep, 7.0},
        {2.8 + sep / 2, 7.0 - radius},
        {2.8 + sep / 2 + sep, 7.0 - radius}};

    std::vector<std::vector<int>> colors = {
        {0, 0, 255}, // Azul
        {0, 0, 0}, // Negro
        {255, 0, 0}, // Rojo
        {255, 255, 0}, // Amarillo
        {0, 255, 0} // Verde
    };

    for (int circle = 0; circle < 5; circle++)
    {
        change_pen(node, client_pen, colors[circle][0], colors[circle][1], colors[circle][2], 5, 1); // Cambiar el color del lápiz y apagarlo.
        teleport_absolute(node, client_teleport, positions[circle][0], positions[circle][1], 0.0); // Teletransportar a la posición inicial.
        change_pen(node, client_pen, colors[circle][0], colors[circle][1], colors[circle][2], 5, 0); // Cambiar el color del lápiz y activarlo.

        int i = 0;
        int n = circle_perimeter/(0.05 * linear_velocity);
        while (rclcpp::ok() and i < n) 
        {
            msg.linear.x = linear_velocity;
            msg.angular.z = angular_velocity;
            publisher->publish(msg);
            rclcpp::spin_some(node);
            loop_rate.sleep();
            i++;
        }

        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        publisher->publish(msg);
        rclcpp::spin_some(node);
    }

    change_pen(node, client_pen, 0, 0, 0, 5, 1); // Cambiar el color del lápiz y apagarlo.
    teleport_absolute(node, client_teleport, 6.0, 4, 0.0);   // Teletransportar a la posición final.

    rclcpp::shutdown();
    return 0;
}