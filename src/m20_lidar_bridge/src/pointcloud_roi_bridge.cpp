#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <unordered_set>

using sensor_msgs::msg::PointCloud2;

namespace
{
struct VoxelKey
{
  int x;
  int y;
  int z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    std::size_t seed = 0;
    auto mix = [&seed](int value) {
      const auto h = std::hash<int>{}(value);
      seed ^= h + 0x9e3779b9U + (seed << 6) + (seed >> 2);
    };
    mix(key.x);
    mix(key.y);
    mix(key.z);
    return seed;
  }
};

int field_offset(const PointCloud2 & msg, const std::string & name)
{
  for (const auto & field : msg.fields) {
    if (field.name == name) {
      return static_cast<int>(field.offset);
    }
  }
  return -1;
}

float read_float_le(const std::vector<uint8_t> & data, std::size_t offset)
{
  float value = 0.0F;
  std::memcpy(&value, data.data() + offset, sizeof(float));
  return value;
}
}  // namespace

class PointCloudRoiBridge : public rclcpp::Node
{
public:
  PointCloudRoiBridge()
  : Node("pointcloud_roi_bridge")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/LIDAR/POINTS");
    output_topic_ = declare_parameter<std::string>("output_topic", "/m20/perception/obstacle_cloud");
    max_rate_hz_ = declare_parameter<double>("max_rate_hz", 10.0);
    stride_ = static_cast<int>(declare_parameter<int>("stride", 1));
    if (stride_ < 1) {
      stride_ = 1;
    }
    max_points_ = static_cast<int>(declare_parameter<int>("max_points", 30000));
    use_roi_ = declare_parameter<bool>("use_roi", true);
    min_x_ = declare_parameter<double>("min_x", -4.0);
    max_x_ = declare_parameter<double>("max_x", 6.0);
    min_y_ = declare_parameter<double>("min_y", -3.0);
    max_y_ = declare_parameter<double>("max_y", 3.0);
    min_z_ = declare_parameter<double>("min_z", -0.6);
    max_z_ = declare_parameter<double>("max_z", 1.8);
    use_voxel_ = declare_parameter<bool>("use_voxel", true);
    voxel_size_ = declare_parameter<double>("voxel_size", 0.10);
    output_frame_ = declare_parameter<std::string>("output_frame", "");
    reliable_qos_ = declare_parameter<bool>("reliable_qos", true);
    log_period_sec_ = declare_parameter<double>("log_period_sec", 2.0);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).durability_volatile();
    if (reliable_qos_) {
      qos.reliable();
    } else {
      qos.best_effort();
    }

    pub_ = create_publisher<PointCloud2>(output_topic_, qos);
    sub_ = create_subscription<PointCloud2>(
      input_topic_, qos, std::bind(&PointCloudRoiBridge::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Bridging %s -> %s, max_rate=%.2fHz, roi=%s, voxel=%.3fm, max_points=%d, qos=%s",
      input_topic_.c_str(), output_topic_.c_str(), max_rate_hz_, use_roi_ ? "on" : "off",
      use_voxel_ ? voxel_size_ : 0.0, max_points_, reliable_qos_ ? "reliable" : "best_effort");
  }

private:
  void cloud_callback(const PointCloud2::SharedPtr msg)
  {
    const auto now = std::chrono::steady_clock::now();
    if (max_rate_hz_ > 0.0 && last_pub_.time_since_epoch().count() != 0) {
      const auto min_period = std::chrono::duration<double>(1.0 / max_rate_hz_);
      if (now - last_pub_ < min_period) {
        return;
      }
    }

    if (msg->is_bigendian) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Big-endian PointCloud2 is not supported.");
      return;
    }

    const int x_offset = field_offset(*msg, "x");
    const int y_offset = field_offset(*msg, "y");
    const int z_offset = field_offset(*msg, "z");
    if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "PointCloud2 lacks x/y/z fields.");
      return;
    }

    if (msg->point_step == 0 || msg->row_step == 0 || msg->width == 0 || msg->height == 0) {
      return;
    }

    PointCloud2 out;
    out.header = msg->header;
    if (!output_frame_.empty()) {
      out.header.frame_id = output_frame_;
    }
    out.height = 1;
    out.fields = msg->fields;
    out.is_bigendian = msg->is_bigendian;
    out.point_step = msg->point_step;
    out.is_dense = false;

    const std::size_t total_points = static_cast<std::size_t>(msg->width) * msg->height;
    const std::size_t reserve_points = max_points_ > 0 ?
      std::min<std::size_t>(static_cast<std::size_t>(max_points_), total_points) : total_points;
    out.data.reserve(reserve_points * msg->point_step);

    std::unordered_set<VoxelKey, VoxelKeyHash> voxels;
    if (use_voxel_ && voxel_size_ > 0.0) {
      voxels.reserve(reserve_points);
    }

    std::size_t seen_points = 0;
    std::size_t selected_points = 0;
    bool done = false;

    for (uint32_t row = 0; row < msg->height && !done; ++row) {
      const std::size_t row_base = static_cast<std::size_t>(row) * msg->row_step;
      for (uint32_t col = 0; col < msg->width; ++col) {
        if ((seen_points++ % static_cast<std::size_t>(stride_)) != 0) {
          continue;
        }

        const std::size_t point_offset = row_base + static_cast<std::size_t>(col) * msg->point_step;
        const std::size_t point_end = point_offset + msg->point_step;
        const auto max_field_offset = static_cast<std::size_t>(
          std::max(x_offset, std::max(y_offset, z_offset)));
        if (point_end > msg->data.size() || point_offset + max_field_offset + sizeof(float) > msg->data.size()) {
          continue;
        }

        const float x = read_float_le(msg->data, point_offset + static_cast<std::size_t>(x_offset));
        const float y = read_float_le(msg->data, point_offset + static_cast<std::size_t>(y_offset));
        const float z = read_float_le(msg->data, point_offset + static_cast<std::size_t>(z_offset));
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }

        if (use_roi_ && (x < min_x_ || x > max_x_ || y < min_y_ || y > max_y_ || z < min_z_ || z > max_z_)) {
          continue;
        }

        if (use_voxel_ && voxel_size_ > 0.0) {
          const VoxelKey key{
            static_cast<int>(std::floor(x / voxel_size_)),
            static_cast<int>(std::floor(y / voxel_size_)),
            static_cast<int>(std::floor(z / voxel_size_))};
          if (!voxels.insert(key).second) {
            continue;
          }
        }

        out.data.insert(out.data.end(), msg->data.begin() + point_offset, msg->data.begin() + point_end);
        ++selected_points;
        if (max_points_ > 0 && selected_points >= static_cast<std::size_t>(max_points_)) {
          done = true;
          break;
        }
      }
    }

    out.width = static_cast<uint32_t>(selected_points);
    out.row_step = out.width * out.point_step;

    pub_->publish(out);
    last_pub_ = now;

    if (last_log_.time_since_epoch().count() == 0 ||
      now - last_log_ > std::chrono::duration<double>(log_period_sec_))
    {
      RCLCPP_INFO(
        get_logger(), "cloud frame=%s input=%zu output=%zu",
        msg->header.frame_id.c_str(), total_points, selected_points);
      last_log_ = now;
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  double max_rate_hz_;
  int stride_;
  int max_points_;
  bool use_roi_;
  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;
  double min_z_;
  double max_z_;
  bool use_voxel_;
  double voxel_size_;
  std::string output_frame_;
  bool reliable_qos_;
  double log_period_sec_;

  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
  std::chrono::steady_clock::time_point last_pub_;
  std::chrono::steady_clock::time_point last_log_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudRoiBridge>());
  rclcpp::shutdown();
  return 0;
}
