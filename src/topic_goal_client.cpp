#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_muxer/action/timer.hpp"

using Timer = action_muxer::action::Timer;
using GoalHandle = rclcpp_action::ClientGoalHandle<Timer>;

class TopicGoalClient : public rclcpp::Node {
public:
    TopicGoalClient() : Node("topic_goal_client") {
        client_ = rclcpp_action::create_client<Timer>(this, "timer");

        // Listen to the topic to send a goal
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "trigger_goal", 10,
            std::bind(&TopicGoalClient::on_topic_msg, this, std::placeholders::_1)
        );
    }

private:
    rclcpp_action::Client<Timer>::SharedPtr client_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    GoalHandle::SharedPtr last_goal_;

    void on_topic_msg(const std_msgs::msg::String::SharedPtr msg) {
        if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(get_logger(), "Action server not ready yet!");
            return;
        }

        // Cancel previous goal
        if (last_goal_) {
            client_->async_cancel_goal(last_goal_);
            RCLCPP_INFO(get_logger(), "Previous goal aborted for: %s", msg->data.c_str());
        }

        Timer::Goal goal_msg;
        goal_msg.id = msg->data;

        auto options = rclcpp_action::Client<Timer>::SendGoalOptions();
        options.feedback_callback = [](auto, const auto & fb) {
            RCLCPP_INFO(rclcpp::get_logger("Feedback"), "Progress: %.1f%%", fb->percentage);
        };
        options.result_callback = [](const GoalHandle::WrappedResult & result) {
            RCLCPP_INFO(rclcpp::get_logger("Result"), "Done? %s", result.result->success ? "Yes ✅" : "No ❌");
        };

        RCLCPP_INFO(this->get_logger(), "Sending goal: %s", goal_msg.id.c_str());
        client_->async_send_goal(goal_msg, options)
            .then([this](auto future) { last_goal_ = future.get(); });
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TopicGoalClient>());
    rclcpp::shutdown();
    return 0;
}
