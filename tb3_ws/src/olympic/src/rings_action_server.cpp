#include <inttypes.h>

#include <memory>

#include "geometry_msgs/msg/twist.hpp"  // This header file defines the Twist message type.
#include "olympic_interfaces/action/rings.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "turtlesim/srv/set_pen.hpp"            // This header file defines the SetPen service type.
#include "turtlesim/srv/teleport_absolute.hpp"  // This header file defines the TeleportAbsolute service type.

bool ninety = false;
bool one_eighty = false;
bool two_seventy = false;
bool three_sixty = false;

using Rings =
    olympic_interfaces::action::Rings;

using GoalHandleRings =
    rclcpp_action::ServerGoalHandle<Rings>;

using namespace std::chrono_literals;

void change_pen(const std::shared_ptr<rclcpp::Node> &node,
                const std::shared_ptr<rclcpp::Client<turtlesim::srv::SetPen>> &client,
                int r, int g, int b, int width, int off) {
    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->r = r;
    request->g = g;
    request->b = b;
    request->width = width;
    request->off = off;

    while (!client->wait_for_service(1s)) {
        if (rclcpp::ok()) {
            RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
        } else {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Set pen color to r:%d, g:%d, b:%d", r, g, b);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service set_pen");
    }
}

void teleport_absolute(const std::shared_ptr<rclcpp::Node> &node,
                       const std::shared_ptr<rclcpp::Client<turtlesim::srv::TeleportAbsolute>> &client,
                       double x, double y, double theta) {
    auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    request->x = x;
    request->y = y;
    request->theta = theta;

    while (!client->wait_for_service(1s)) {
        if (rclcpp::ok()) {
            RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
        } else {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Teleported to x:%f, y:%f, theta:%f", x, y, theta);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service teleport_absolute");
    }
}

rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Rings::Goal> goal) {
    RCLCPP_INFO(rclcpp::get_logger("server"),
                "Got goal request with order %f", goal->radius);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleRings> goal_handle) {
    RCLCPP_INFO(rclcpp::get_logger("server"),
                "Got request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void execute(const std::shared_ptr<GoalHandleRings> goal_handle);

void handle_accepted(const std::shared_ptr<GoalHandleRings> goal_handle) {
    std::thread{execute, goal_handle}.detach();
}

void execute(const std::shared_ptr<GoalHandleRings> goal_handle) {
    RCLCPP_INFO(rclcpp::get_logger("server"),
                "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Rings::Feedback>();
    auto result = std::make_shared<Rings::Result>();
    auto temp_node = rclcpp::Node::make_shared("temp_node");  // Nodo temporal
    auto publisher = temp_node->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    auto client_pen = temp_node->create_client<turtlesim::srv::SetPen>("turtle1/set_pen");
    auto client_teleport = temp_node->create_client<turtlesim::srv::TeleportAbsolute>("turtle1/teleport_absolute");
    geometry_msgs::msg::Twist msg;
    rclcpp::WallRate loop_rate(50ms);

    double radius = goal->radius;  // Obtener el radio del objetivo.
    double linear_velocity = 1.0;
    double angular_velocity = linear_velocity / radius;  // Calcular la velocidad angular basada en la velocidad lineal y el radio.

    double circle_perimeter = 2 * M_PI * radius;  // Calcular el perímetro del círculo.
    int completed_rings = 0;
    double sep = radius * 2.4;

    std::vector<std::vector<double>> positions = {
        {2.8, 7.0},
        {2.8 + sep, 7.0},
        {2.8 + 2 * sep, 7.0},
        {2.8 + sep / 2, 7.0 - radius},
        {2.8 + sep / 2 + sep, 7.0 - radius}};

    std::vector<std::vector<int>> colors = {
        {0, 0, 255},    // Azul
        {0, 0, 0},      // Negro
        {255, 0, 0},    // Rojo
        {255, 255, 0},  // Amarillo
        {0, 255, 0}     // Verde
    };

    for (int circle = 0; circle < 5; circle++) {
        change_pen(temp_node, client_pen, colors[circle][0], colors[circle][1], colors[circle][2], 5, 1);  // Cambiar el color del lápiz y apagarlo.
        teleport_absolute(temp_node, client_teleport, positions[circle][0], positions[circle][1], 0.0);    // Teletransportar a la posición inicial.
        change_pen(temp_node, client_pen, colors[circle][0], colors[circle][1], colors[circle][2], 5, 0);  // Cambiar el color del lápiz y activarlo.

        int i = 0;
        int n = circle_perimeter / (0.05 * linear_velocity);
        while (rclcpp::ok() and i < n) {
            msg.linear.x = linear_velocity;
            msg.angular.z = angular_velocity;
            publisher->publish(msg);
            float completed_angle = 360.0 * i / n;
            if (completed_angle > 85 && completed_angle < 95 && !ninety) {
                feedback->ring_angle = 90;
                ninety = true;
                one_eighty = false;
                two_seventy = false;
                three_sixty = false;
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(rclcpp::get_logger("server"),
                            "Publish Feedback");
            } else if (completed_angle > 175 && completed_angle < 185 && !one_eighty) {
                feedback->ring_angle = 180;
                ninety = false;
                one_eighty = true;
                two_seventy = false;
                three_sixty = false;
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(rclcpp::get_logger("server"),
                            "Publish Feedback");
            } else if (completed_angle > 265 && completed_angle < 275 && !two_seventy) {
                feedback->ring_angle = 270;
                ninety = false;
                one_eighty = false;
                two_seventy = true;
                three_sixty = false;
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(rclcpp::get_logger("server"),
                            "Publish Feedback");
            } else if (completed_angle > 355 && completed_angle < 365 && !three_sixty) {
                feedback->ring_angle = 360;
                ninety = false;
                one_eighty = false;
                two_seventy = false;
                three_sixty = true;
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(rclcpp::get_logger("server"),
                            "Publish Feedback");
            }

            loop_rate.sleep();
            i++;
        }

        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        publisher->publish(msg);
        completed_rings++;
        feedback->drawing_ring = completed_rings;
        feedback->ring_angle = 0;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(rclcpp::get_logger("server"),
                    "Publish Feedback");
    }

    change_pen(temp_node, client_pen, 0, 0, 0, 5, 1);            // Cambiar el color del lápiz y apagarlo.
    teleport_absolute(temp_node, client_teleport, 6.0, 4, 0.0);  // Teletransportar a la posición final.

    if (rclcpp::ok()) {
        result->rings_completed = completed_rings;
        goal_handle->succeed(result);
        RCLCPP_INFO(rclcpp::get_logger("server"),
                    "Goal Succeeded");
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("action_server");
    auto action_server = rclcpp_action::create_server<Rings>(node,
                                                             "rings",
                                                             handle_goal,
                                                             handle_cancel,
                                                             handle_accepted);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
