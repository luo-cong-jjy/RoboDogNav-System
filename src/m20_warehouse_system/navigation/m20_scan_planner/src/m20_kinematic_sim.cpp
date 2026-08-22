/**
 * @file m20_kinematic_sim.cpp
 * @brief M20 运动学仿真节点（M20KinematicSim）
 *
 * 职责：
 *  - 订阅速度指令（cmd_vel，Twist），对指令做限幅与超时保护（cmd_timeout 内未
 *    收到新指令则自动停车）；
 *  - 按二维运动学模型积分机体位姿：世界系速度由机体系指令经偏航角旋转得到，
 *    位置累加、偏航角累加（归一化到 [-π, π]）；
 *  - 以固定频率（默认 100 Hz）发布机体位姿（body_pose，Odometry），
 *    可选广播 TF（odom -> base，publish_tf）。
 *
 * 用于在无 Gazebo 的情况下快速仿真底盘运动，供规划/导航管线联调。
 */
#include <algorithm>    // 标准算法库：std::clamp / std::min
#include <cmath>        // 数学库：std::cos / std::sin / M_PI
#include <memory>       // 智能指针：std::make_shared / std::make_unique
#include <string>       // 字符串：std::string（坐标系名）

#include <geometry_msgs/msg/transform_stamped.hpp>   // TF 消息：TransformStamped（发布 TF 用）
#include <geometry_msgs/msg/twist.hpp>               // 速度消息：Twist（订阅 cmd_vel）
#include <nav_msgs/msg/odometry.hpp>                 // 里程计消息：Odometry（发布 body_pose）
#include <rclcpp/rclcpp.hpp>                         // ROS2 C++ 客户端库：Node/Publisher/Timer
#include <tf2/LinearMath/Quaternion.h>               // TF2 四元数：由 RPY 构造姿态
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>   // TF2 与 geometry_msgs 转换（新版本路径）
#else
// ROS 2 Foxy installs this compatibility header with the legacy suffix.
// ROS 2 Foxy 安装的是带旧后缀的兼容头文件
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <tf2_ros/transform_broadcaster.h>   // TF2 广播器：广播 odom->base 变换

namespace scan_planner    // 扫描规划器命名空间
{
// M20 运动学仿真节点：按运动学模型积分并发布机体位姿
class M20KinematicSim : public rclcpp::Node
{
public:
  // 构造函数：声明参数、创建订阅/发布/TF 广播器/定时器
  M20KinematicSim() : Node("m20_kinematic_sim")
  {
    x_ = declare_parameter<double>("init_x", 0.0);        // 初始位置 x
    y_ = declare_parameter<double>("init_y", 0.0);        // 初始位置 y
    z_ = declare_parameter<double>("init_z", 0.3);        // 初始高度 z
    yaw_ = declare_parameter<double>("init_yaw", 0.0);    // 初始偏航角
    max_vx_ = declare_parameter<double>("max_vx", 0.75);  // 最大前进速度 [m/s]
    max_vy_ = declare_parameter<double>("max_vy", 0.35);  // 最大侧向速度 [m/s]
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);   // 最大偏航角速度（带硬上限）
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.3);   // 指令超时时间 [s]
    const double sim_rate = declare_parameter<double>("sim_rate", 100.0);   // 仿真/发布频率 [Hz]
    publish_tf_ = declare_parameter<bool>("publish_tf", false);      // 是否广播 TF
    frame_id_ = declare_parameter<std::string>("frame_id", "world"); // 里程计父坐标系
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base");   // 机体子坐标系

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);   // 创建 TF 广播器
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("body_pose", 100);    // 发布机体位姿
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(                  // 订阅速度指令
        "cmd_vel", 20, std::bind(&M20KinematicSim::cmdCallback, this, std::placeholders::_1));
    last_cmd_time_ = now();        // 初始化指令时间戳
    last_sim_time_ = now();        // 初始化仿真时间戳
    timer_ = create_wall_timer(    // 周期定时器：按仿真频率驱动积分与发布
        std::chrono::duration<double>(1.0 / std::max(1.0, sim_rate)),
        std::bind(&M20KinematicSim::simCallback, this));
    RCLCPP_INFO(get_logger(), "M20 kinematic simulator ready");   // 打印就绪信息
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;   // 偏航角速度硬上限 [rad/s]

  // 将角度归一化到 [-π, π]
  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;    // 超过上界则减一整圈
    while (angle < -M_PI) angle += 2.0 * M_PI;   // 低于下界则加一整圈
    return angle;                                // 返回归一化角度
  }

  // 速度指令回调：对线/角速度限幅并记录指令时刻
  void cmdCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    vx_cmd_ = std::clamp(msg->linear.x, -max_vx_, max_vx_);      // 前进速度限幅
    vy_cmd_ = std::clamp(msg->linear.y, -max_vy_, max_vy_);      // 侧向速度限幅
    vyaw_cmd_ = std::clamp(msg->angular.z, -max_vyaw_, max_vyaw_);   // 偏航角速度限幅
    last_cmd_time_ = now();                      // 记录收到指令的时刻
  }

  // 发布机体位姿（Odometry）与可选 TF
  void publishOdom(const rclcpp::Time &stamp)
  {
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw_);           // 由 RPY 构造姿态（仅偏航）
    const auto orientation = tf2::toMsg(quaternion);   // 四元数转消息
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;             // 时间戳
    odom.header.frame_id = frame_id_;      // 父坐标系
    odom.child_frame_id = child_frame_id_; // 子坐标系
    odom.pose.pose.position.x = x_;        // 位置 x
    odom.pose.pose.position.y = y_;        // 位置 y
    odom.pose.pose.position.z = z_;        // 位置 z
    odom.pose.pose.orientation = orientation;   // 姿态
    odom.twist.twist.linear.x = vx_world_;  // 世界系线速度 x
    odom.twist.twist.linear.y = vy_world_;  // 世界系线速度 y
    odom.twist.twist.angular.z = vyaw_cmd_; // 偏航角速度（取指令值）
    odom_pub_->publish(odom);               // 发布位姿消息

    if (publish_tf_)                        // 允许广播 TF
    {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;       // 复用里程计头（时间戳/父系）
      transform.child_frame_id = child_frame_id_;   // 子坐标系
      transform.transform.translation.x = x_;   // 平移 x
      transform.transform.translation.y = y_;   // 平移 y
      transform.transform.translation.z = z_;   // 平移 z
      transform.transform.rotation = orientation;   // 旋转
      tf_broadcaster_->sendTransform(transform);    // 广播 TF 变换
    }
  }

  // 仿真定时器回调：积分运动学模型并发布位姿
  void simCallback()
  {
    const auto current_time = now();       // 当前时刻
    double dt = (current_time - last_sim_time_).seconds();   // 计算积分步长
    last_sim_time_ = current_time;         // 更新上次仿真时刻
    if (dt < 0.0 || dt > 0.2) dt = 0.0;    // 步长非法/过大则置 0（暂停恢复保护）
    double vx = vx_cmd_, vy = vy_cmd_, wz = vyaw_cmd_;   // 取当前指令
    if ((current_time - last_cmd_time_).seconds() > cmd_timeout_)   // 指令超时
      vx = vy = wz = 0.0;                  // 自动停车（指令清零）
    const double c = std::cos(yaw_);       // 偏航角余弦
    const double s = std::sin(yaw_);       // 偏航角正弦
    vx_world_ = c * vx - s * vy;           // 坐标变换：机体系 -> 世界系 x 速度
    vy_world_ = s * vx + c * vy;           // 坐标变换：机体系 -> 世界系 y 速度
    x_ += vx_world_ * dt;                  // 积分位置 x
    y_ += vy_world_ * dt;                  // 积分位置 y
    yaw_ = normalizeAngle(yaw_ + wz * dt); // 积分偏航角并归一化
    publishOdom(current_time);             // 发布机体位姿
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;   // body_pose 发布器
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;   // cmd_vel 订阅器
  rclcpp::TimerBase::SharedPtr timer_;     // 仿真定时器
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;   // TF 广播器
  double x_{0.0}, y_{0.0}, z_{0.3}, yaw_{0.0};   // 机体位置与偏航角状态
  double vx_cmd_{0.0}, vy_cmd_{0.0}, vyaw_cmd_{0.0};   // 指令速度（机体系）
  double vx_world_{0.0}, vy_world_{0.0};     // 世界系速度（由指令旋转得到）
  double max_vx_{0.75}, max_vy_{0.35}, max_vyaw_{1.0}, cmd_timeout_{0.3};   // 限幅与超时参数
  bool publish_tf_{false};                   // 是否广播 TF
  std::string frame_id_, child_frame_id_;    // 坐标系名
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};   // 最近指令时刻
  rclcpp::Time last_sim_time_{0, 0, RCL_ROS_TIME};   // 最近仿真时刻
};
}  // namespace scan_planner

int main(int argc, char **argv)   // 程序入口
{
  rclcpp::init(argc, argv);       // 初始化 ROS2 运行时
  rclcpp::spin(std::make_shared<scan_planner::M20KinematicSim>());   // 创建节点并阻塞式 spin
  rclcpp::shutdown();             // 关闭 ROS2 运行时
  return 0;                       // 正常退出
}
