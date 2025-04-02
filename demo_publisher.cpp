#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

class GoalPublisher : public rclcpp::Node {
public:
    GoalPublisher() : Node("goal_publisher") {
        publisher_ = create_publisher<std_msgs::msg::Int32>("goal_request", 10);
        timer_ = create_wall_timer(
            std::chrono::seconds(2),
            [this]() {
                auto message = std_msgs::msg::Int32();
                message.data = count_++;
                publisher_->publish(message);
                RCLCPP_INFO(get_logger(), "Published goal request: %d", message.data);
            });
    }

private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_ = 1;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
