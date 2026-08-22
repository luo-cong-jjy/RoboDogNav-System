/**
 * @file scan_planner_node.cpp
 * @brief 扫描规划器主节点入口（scan_planner_node）
 *
 * 职责：
 *  - 创建 ROS2 节点 scan_planner_node；
 *  - 实例化规划状态机（SCANReplanFSM）并调用 init 完成参数加载、
 *    模块初始化和订阅/发布/服务创建；
 *  - 用单线程执行器（SingleThreadedExecutor）spin 驱动 FSM 与安全检测定时器；
 *  - 初始化失败（如导航模式参数非法）时打印致命错误并返回非零退出码。
 */
#include <memory>        // 智能指针：std::make_shared
#include <exception>     // 异常类型：std::exception（初始化失败捕获）

#include <rclcpp/rclcpp.hpp>              // ROS2 C++ 客户端库：Node/Executor
#include <plan_manage/scan_replan_fsm.h>  // 规划状态机：SCANReplanFSM

int main(int argc, char **argv)   // 程序入口
{
  rclcpp::init(argc, argv);       // 初始化 ROS2 运行时
  auto node = std::make_shared<rclcpp::Node>("scan_planner_node");   // 创建主节点

  try                              // 尝试初始化规划器
  {
    scan_planner::SCANReplanFSM planner;   // 创建规划状态机对象
    planner.init(node.get());      // 初始化状态机（读参数/建模块/建订阅发布）
    rclcpp::executors::SingleThreadedExecutor executor;   // 单线程执行器：串行处理回调
    executor.add_node(node);       // 将节点加入执行器
    executor.spin();               // 阻塞式 spin，运行直到 shutdown
  }
  catch (const std::exception &error)   // 捕获初始化异常
  {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize SCAN-Planner: %s", error.what());   // 打印致命错误
    rclcpp::shutdown();            // 关闭 ROS2 运行时
    return 1;                      // 返回失败退出码
  }

  rclcpp::shutdown();              // 正常关闭 ROS2 运行时
  return 0;                        // 正常退出
}
