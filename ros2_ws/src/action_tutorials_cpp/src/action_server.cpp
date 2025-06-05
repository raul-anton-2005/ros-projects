#include <inttypes.h>
#include <memory>
#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using Fibonacci =
    action_tutorials_interfaces::action::Fibonacci;

using GoalHandleFibonacci =
    rclcpp_action::ServerGoalHandle<Fibonacci>;

rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
{
    RCLCPP_INFO(rclcpp::get_logger("server"),
                "Got goal request with order %d", goal->order);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
    RCLCPP_INFO(rclcpp::get_logger("server"),
                "Got request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void execute(const std::shared_ptr<GoalHandleFibonacci>);

void handle_accepted(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
    std::thread{execute, goal_handle}.detach();
}

void execute(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
    RCLCPP_INFO(rclcpp::get_logger("server"),
                "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto &sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i)
    {
        if (goal_handle->is_canceling())
        {
            result->sequence = sequence;
            goal_handle->canceled(result);
            RCLCPP_INFO(rclcpp::get_logger("server"),
                        "Goal Canceled");
            return;
        }
        sequence.push_back(sequence[i] + sequence[i - 1]);
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(rclcpp::get_logger("server"),
                    "Publish Feedback");
        loop_rate.sleep();
    }

    if (rclcpp::ok())

    {
        result->sequence = sequence;
        goal_handle->succeed(result);
        RCLCPP_INFO(rclcpp::get_logger("server"),
                    "Goal Succeeded");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("action_server");
    auto action_server =
        rclcpp_action::create_server<Fibonacci>(node,
                                                "fibonacci",
                                                handle_goal,
                                                handle_cancel,
                                                handle_accepted);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
