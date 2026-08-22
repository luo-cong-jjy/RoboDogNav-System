/**
 * @file line_demo.cpp
 * @brief 直线运动演示节点（LineDemo）
 *
 * 职责：
 *  - 以固定频率（默认 20 Hz）向 /cmd_vel 话题发布恒定线速度指令（默认 -0.20 m/s），
 *    使机器人沿直线运动一段设定时长（默认 8 s）后自动停车并关闭节点；
 *  - 用于在 Gazebo 仿真中验证底盘直线运动、里程计与导航管线；
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

// 直线运动演示节点：周期性地发布恒定线速度指令
class LineDemo : public rclcpp::Node {
public:
    // 构造函数：声明参数、创建发布器与定时器
    LineDemo() : Node("m20_line_demo") {
        topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");   // 速度指令话题名
        speed_ = declare_parameter<double>("speed", -0.20);                     // 直线运动线速度 [m/s]
        duration_sec_ = declare_parameter<double>("duration_sec", 8.0);         // 运动持续时间 [s]
        rate_hz_ = declare_parameter<double>("rate_hz", 20.0);                  // 发布频率 [Hz]

        pub_ = create_publisher<geometry_msgs::msg::Twist>(topic_, 10);         // 创建 /cmd_vel 发布器
        start_time_ = now();                                                    // 记录启动时刻（用于计算已运行时长）

        const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_));   // 由发布频率换算定时器周期
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),   // 创建周期定时器
                                   std::bind(&LineDemo::OnTimer, this));   // 绑定定时器回调

        RCLCPP_INFO(get_logger(), "Line demo: topic=%s speed=%.3f duration=%.2fs",   // 打印演示配置信息
                    topic_.c_str(), speed_, duration_sec_);
    }

private:
    // 定时器回调：按剩余时间发布速度指令，超时后发零速并计数，随后关闭节点
    void OnTimer() {
        const double elapsed = (now() - start_time_).seconds();   // 距启动已过的秒数

        geometry_msgs::msg::Twist cmd;    // 待发布的速度指令
        if (elapsed < duration_sec_) {     // 仍在运动时间内
            cmd.linear.x = speed_;         // 保持恒定线速度
        } else {                           // 超过运动时间
            cmd.linear.x = 0.0;            // 发送零速指令（停车）
            stop_count_++;                 // 累计停车发布次数
        }

        pub_->publish(cmd);                // 发布速度指令到 /cmd_vel

        if (stop_count_ > 20) {            // 已连续发布 20 次以上零速（约 1 s）
            RCLCPP_INFO(get_logger(), "Line demo finished.");   // 打印演示结束信息
            rclcpp::shutdown();            // 关闭 ROS2 运行时，结束节点
        }
    }

    std::string topic_;                    // 速度指令话题名
    double speed_ = 0.20;                  // 线速度 [m/s]
    double duration_sec_ = 8.0;            // 运动持续时间 [s]
    double rate_hz_ = 20.0;                // 发布频率 [Hz]
    int stop_count_ = 0;                   // 零速指令连续发布次数

    rclcpp::Time start_time_;                                              // 节点启动时刻
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;          // /cmd_vel 发布器
    rclcpp::TimerBase::SharedPtr timer_;                                   // 周期定时器
};

int main(int argc, char** argv) {          // 程序入口
    rclcpp::init(argc, argv);              // 初始化 ROS2 运行时
    rclcpp::spin(std::make_shared<LineDemo>());   // 创建节点并阻塞式 spin（处理回调直到 shutdown）
    return 0;                              // 正常退出
}
