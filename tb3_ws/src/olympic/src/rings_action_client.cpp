#include <inttypes.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"         // This header file defines the Twist message type.
#include "turtlesim/srv/set_pen.hpp"           // This header file defines the SetPen service type.
#include "turtlesim/srv/teleport_absolute.hpp" // This header file defines the TeleportAbsolute service type.
#include "olympic_interfaces/action/rings.hpp"

using Rings =
    olympic_interfaces::action::Rings;

using GoalHandleRings =
    rclcpp_action::ClientGoalHandle<Rings>;

using namespace std::chrono_literals;

rclcpp::Node::SharedPtr g_node = nullptr;

void feedback_callback(GoalHandleRings::SharedPtr, const std::shared_ptr<const Rings::Feedback> feedback)
{
    RCLCPP_INFO(g_node->get_logger(), "Rings completed: %d", feedback->drawing_ring);
    
    RCLCPP_INFO(g_node->get_logger(), "Ring angle: %f", feedback->ring_angle);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("action_client");
    auto action_client = rclcpp_action::create_client<Rings>(g_node, "rings");

    g_node->declare_parameter("radius", 1.0);
    double radius = g_node->get_parameter("radius").get_parameter_value().get<double>();

    if (!action_client->wait_for_action_server(20s))
    {
        RCLCPP_ERROR(g_node->get_logger(), "Action server not available after waiting");
        return 1;
    }
    auto goal_msg = Rings::Goal();
    goal_msg.radius = radius;

    RCLCPP_INFO(g_node->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Rings>::SendGoalOptions();
    send_goal_options.feedback_callback = feedback_callback;
    auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);
    auto return_code = rclcpp::spin_until_future_complete(g_node, goal_handle_future);

    if (return_code != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(g_node->get_logger(),
                    "send goal call failed :(");
        return 1;
    }

    GoalHandleRings::SharedPtr goal_handle = goal_handle_future.get();

    if (!goal_handle)
    {
        RCLCPP_ERROR(g_node->get_logger(),
                    "Goal was rejected by server");
        return 1;
    }

    auto result_future = action_client->async_get_result(goal_handle);

    RCLCPP_INFO(g_node->get_logger(), "Waiting for result");

    return_code = rclcpp::spin_until_future_complete(g_node, result_future);

    if (return_code != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(g_node->get_logger(), "get result call failed :(");
        return 1;
    }

    GoalHandleRings::WrappedResult wrapped_result = result_future.get();

    switch (wrapped_result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(g_node->get_logger(), "Goal was aborted");
        return 1;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(g_node->get_logger(), "Goal was canceled");
        return 1;
    default:
        RCLCPP_ERROR(g_node->get_logger(), "Unknown result code");
        return 1;
    }

    RCLCPP_INFO(g_node->get_logger(), "result received");
    RCLCPP_INFO(g_node->get_logger(), "%" PRId32, wrapped_result.result->rings_completed);

    action_client.reset();
    g_node.reset();
    rclcpp::shutdown();
    return 0;
}