#ifndef CLOUD_TO_DEPTH
#define CLOUD_TO_DEPTH

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include "native_adapters/PCL_2.hpp"

namespace perception {

class CloudToDepth : public rclcpp::Node {
 public:
  explicit CloudToDepth(const rclcpp::NodeOptions & options);

  ~CloudToDepth() {}

 private:
  std::string source_topic, dest_topic;
  bool as_float = false;

  using Adapter = rclcpp::adapt_type<StampedPointCloud2>::as<sensor_msgs::msg::PointCloud2>;

  rclcpp::Subscription<Adapter>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

  void callback_pointcloud(const std::shared_ptr<const StampedPointCloud2> &msg);

  template <typename PointT>
  void process_message(const std_msgs::msg::Header &header, const pcl::PointCloud<PointT> &msg);
};

}  // namespace perception

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(perception::CloudToDepth)

#endif  // CLOUD_TO_DEPTH