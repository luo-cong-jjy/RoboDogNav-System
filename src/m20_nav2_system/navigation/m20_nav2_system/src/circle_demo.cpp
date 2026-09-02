/**
 * @file circle_demo.cpp
 * @brief 圆周运动演示节点（CircleDemo）
 *
 * 职责：
 *  - 以固定频率（默认 20 Hz）向 /cmd_vel 话题发布"线速度 + 角速度"组合指令
 *    （默认 0.20 m/s 与 0.30 rad/s），使机器人沿近似圆周运动；
 *  - 圆周半径约为 speed / yaw_rate，运动设定时长（默认 15 s）后自动停车并关闭节点；
 *  - 用于在 Gazebo 仿真中验证底盘转弯/角速度控制与导航管线；
 *  - 所有参数均通过 ROS2 参数声明，可在 launch 文件或命令行中覆盖。
 */
#include <chrono>       // 时间库：std::chrono（定时器周期换算）
#include <algorithm>    // 标准算法库：std::max（频率下限保护）
#include <functional>   // 函数对象：std::bind（绑定定时器回调）
#include <memory>       // 智能指针：std::make_shared
#include <string>       // 字符串：std::string（话题名参数）

#include "geometry_msgs/msg/twist.hpp"   // 速度消息：Twist（发布到 /cmd_vel）
#include "rclcpp/rclcpp.hpp"             // ROS2 C++ 客户端库：Node/Publisher/Timer

using namespace std::chrono_literals;    // 时间字面量（毫秒等单位）

// 圆周运动演示节点：周期性地发布线速度 + 角速度组合指令
class CircleDemo : public rclcpp::Node {
public:
    // 构造函数：声明参数、创建发布器与定时器
    CircleDemo() : Node("m20_circle_demo") {
        topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");   // 速度指令话题名
        speed_ = declare_parameter<double>("speed", 0.20);                      // 线速度 [m/s]
        yaw_rate_ = declare_parameter<double>("yaw_rate", 0.30);                // 偏航角速度 [rad/s]
        duration_sec_ = declare_parameter<double>("duration_sec", 15.0);        // 运动持续时间 [s]
        rate_hz_ = declare_parameter<double>("rate_hz", 20.0);                  // 发布频率 [Hz]

        pub_ = create_publisher<geometry_msgs::msg::Twist>(topic_, 10);         // 创建 /cmd_vel 发布器
        start_time_ = now();                                                    // 记录启动时刻（用于计算已运行时长）

        const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_));   // 由发布频率换算定时器周期
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),   // 创建周期定时器
                                   std::bind(&CircleDemo::OnTimer, this));   // 绑定定时器回调

        RCLCPP_INFO(get_logger(), "Circle demo: topic=%s speed=%.3f yaw_rate=%.3f radius~=%.3fm duration=%.2fs",   // 打印演示配置信息
                    topic_.c_str(), speed_, yaw_rate_,
                    yaw_rate_ == 0.0 ? 0.0 : speed_ / yaw_rate_, duration_sec_);   // 圆周半径 ≈ v / ω（yaw_rate 为 0 时半径为 0）
    }

private:
    // 定时器回调：按剩余时间发布速度指令，超时后发零速并计数，随后关闭节点
    void OnTimer() {
        const double elapsed = (now() - start_time_).seconds();   // 距启动已过的秒数

        geometry_msgs::msg::Twist cmd;    // 待发布的速度指令
        if (elapsed < duration_sec_) {     // 仍在运动时间内
            cmd.linear.x = speed_;         // 保持恒定线速度
            cmd.angular.z = yaw_rate_;     // 保持恒定偏航角速度
        } else {                           // 超过运动时间
            cmd.linear.x = 0.0;            // 线速度置零
            cmd.angular.z = 0.0;           // 角速度置零
            stop_count_++;                 // 累计停车发布次数
        }

        pub_->publish(cmd);                // 发布速度指令到 /cmd_vel

        if (stop_count_ > 20) {            // 已连续发布 20 次以上零速（约 1 s）
            RCLCPP_INFO(get_logger(), "Circle demo finished.");   // 打印演示结束信息
            rclcpp::shutdown();            // 关闭 ROS2 运行时，结束节点
        }
    }

    std::string topic_;                    // 速度指令话题名
    double speed_ = 0.20;                  // 线速度 [m/s]
    double yaw_rate_ = 0.30;               // 偏航角速度 [rad/s]
    double duration_sec_ = 15.0;           // 运动持续时间 [s]
    double rate_hz_ = 20.0;                // 发布频率 [Hz]
    int stop_count_ = 0;                   // 零速指令连续发布次数

    rclcpp::Time start_time_;                                              // 节点启动时刻
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;          // /cmd_vel 发布器
    rclcpp::TimerBase::SharedPtr timer_;                                   // 周期定时器
};

int main(int argc, char** argv) {          // 程序入口
    rclcpp::init(argc, argv);              // 初始化 ROS2 运行时
    rclcpp::spin(std::make_shared<CircleDemo>());   // 创建节点并阻塞式 spin（处理回调直到 shutdown）
    return 0;                              // 正常退出
}
