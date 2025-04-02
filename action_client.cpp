#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "custom_interfaces/action/fibonacci.hpp"

class FibonacciActionClient : public rclcpp::Node {
public:
    using Fibonacci = custom_interfaces::action::Fibonacci;
    using GoalHandle = rclcpp_action::ClientGoalHandle<Fibonacci>;

    explicit FibonacciActionClient() : Node("fibonacci_action_client") {
        client_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");
        timer_ = create_wall_timer(
            std::chrono::seconds(3),
            [this]() { send_goal(); });
    }

    void send_goal() {
        if (!client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_ERROR(get_logger(), "Action server not available");
            return;
        }

        auto goal = Fibonacci::Goal();
        goal.order = 5;

        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            [this](const GoalHandle::SharedPtr & goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(get_logger(), "Goal rejected");
                } else {
                    RCLCPP_INFO(get_logger(), "Goal accepted");
                }
            };
        
        send_goal_options.feedback_callback =
            [this](GoalHandle::SharedPtr, const std::shared_ptr<const Fibonacci::Feedback> feedback) {
                RCLCPP_INFO(get_logger(), "Next number: %d", feedback->partial_sequence.back());
            };

        send_goal_options.result_callback =
            [this](const GoalHandle::WrappedResult & result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(get_logger(), "Result received");
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(get_logger(), "Goal aborted");
                        return;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_ERROR(get_logger(), "Goal canceled");
                        return;
                    default:
                        RCLCPP_ERROR(get_logger(), "Unknown result code");
                        return;
                }
            };

        client_->async_send_goal(goal, send_goal_options);
    }

private:
    rclcpp_action::Client<Fibonacci>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FibonacciActionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
