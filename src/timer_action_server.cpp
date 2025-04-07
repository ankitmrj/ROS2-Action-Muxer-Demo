#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_muxer/action/timer.hpp"

using Timer = action_muxer::action::Timer;
using GoalHandle = rclcpp_action::ServerGoalHandle<Timer>;

class TimerServer : public rclcpp::Node {
public:
    TimerServer() : Node("timer_action_server") {
        server_ = rclcpp_action::create_server<Timer>(
            this,
            "timer",
            std::bind(&TimerServer::on_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&TimerServer::on_cancel, this, std::placeholders::_1),
            std::bind(&TimerServer::on_accept, this, std::placeholders::_1)
        );
    }

private:
    rclcpp_action::Server<Timer>::SharedPtr server_;

    rclcpp_action::GoalResponse on_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const Timer::Goal> goal) {
        RCLCPP_INFO(get_logger(), "New goal: %s", goal->id.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse on_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_WARN(get_logger(), "Cancel requested");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void on_accept(const std::shared_ptr<GoalHandle> goal_handle) {
        std::thread{[this, goal_handle]() { run_goal(goal_handle); }}.detach();
    }

    void run_goal(const std::shared_ptr<GoalHandle> goal_handle) {
        auto goal = goal_handle->get_goal();
        rclcpp::Rate loop(1); // 1 Hz = 1 second

        for (int i = 1; i <= 5; ++i) {
            if (goal_handle->is_canceling()) {
                goal_handle->canceled(std::make_shared<Timer::Result>());
                RCLCPP_INFO(this->get_logger(), "Goal cancelled midway");
                return;
            }
            auto feedback = std::make_shared<Timer::Feedback>();
            feedback->percentage = i * 20.0;
            goal_handle->publish_feedback(feedback);
            loop.sleep();
        }

        auto result = std::make_shared<Timer::Result>();
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal done ✅");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TimerServer>());
    rclcpp::shutdown();
    return 0;
}
