#include <chrono>
#include <algorithm>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class LineDemo : public rclcpp::Node {
public:
    LineDemo() : Node("m20_line_demo") {
        topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        speed_ = declare_parameter<double>("speed", -0.20);
        duration_sec_ = declare_parameter<double>("duration_sec", 8.0);
        rate_hz_ = declare_parameter<double>("rate_hz", 20.0);

        pub_ = create_publisher<geometry_msgs::msg::Twist>(topic_, 10);
        start_time_ = now();

        const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_));
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                                   std::bind(&LineDemo::OnTimer, this));

        RCLCPP_INFO(get_logger(), "Line demo: topic=%s speed=%.3f duration=%.2fs",
                    topic_.c_str(), speed_, duration_sec_);
    }

private:
    void OnTimer() {
        const double elapsed = (now() - start_time_).seconds();

        geometry_msgs::msg::Twist cmd;
        if (elapsed < duration_sec_) {
            cmd.linear.x = speed_;
        } else {
            cmd.linear.x = 0.0;
            stop_count_++;
        }

        pub_->publish(cmd);

        if (stop_count_ > 20) {
            RCLCPP_INFO(get_logger(), "Line demo finished.");
            rclcpp::shutdown();
        }
    }

    std::string topic_;
    double speed_ = 0.20;
    double duration_sec_ = 8.0;
    double rate_hz_ = 20.0;
    int stop_count_ = 0;

    rclcpp::Time start_time_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineDemo>());
    return 0;
}
