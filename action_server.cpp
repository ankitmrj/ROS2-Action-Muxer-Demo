#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/int32.hpp>
#include <memory>
#include "custom_interfaces/action/fibonacci.hpp"

using Fibonacci = custom_interfaces::action::Fibonacci;
using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

class FibonacciActionServer : public rclcpp::Node {
public:
    FibonacciActionServer() : Node("fibonacci_action_server") {
        // Action server
        action_server_ = rclcpp_action::create_server<Fibonacci>(
            this,
            "fibonacci",
            std::bind(&FibonacciActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&FibonacciActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&FibonacciActionServer::handle_accepted, this, std::placeholders::_1));

        // Topic subscriber for goal requests
        goal_subscriber_ = create_subscription<std_msgs::msg::Int32>(
            "goal_request", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                if (current_goal_handle_ && current_goal_handle_->is_active()) {
                    auto result = std::make_shared<Fibonacci::Result>();
                    current_goal_handle_->abort(result);
                    RCLCPP_INFO(get_logger(), "Preempting current goal for new request: %d", msg->data);
                }
            });
    }

private:
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr goal_subscriber_;
    std::shared_ptr<GoalHandle> current_goal_handle_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const Fibonacci::Goal> goal) {
        RCLCPP_INFO(get_logger(), "Received goal request: %d", goal->order);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle>) {
        RCLCPP_INFO(get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
        current_goal_handle_ = goal_handle;
        std::thread{std::bind(&FibonacciActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle) {
        auto result = std::make_shared<Fibonacci::Result>();
        auto feedback = std::make_shared<Fibonacci::Feedback>();
        feedback->partial_sequence = {0, 1};

        rclcpp::Rate loop_rate(1);
        
        for (int i = 1; i <= goal_handle->get_goal()->order && rclcpp::ok(); ++i) {
            if (goal_handle->is_canceling()) {
                result->sequence = feedback->partial_sequence;
                goal_handle->canceled(result);
                RCLCPP_INFO(get_logger(), "Goal canceled");
                return;
            }

            if (i > 1) {
                feedback->partial_sequence.push_back(
                    feedback->partial_sequence[i-1] + feedback->partial_sequence[i-2]);
            }

            goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
        }

        if (goal_handle->is_active()) {
            result->sequence = feedback->partial_sequence;
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Goal succeeded");
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FibonacciActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
