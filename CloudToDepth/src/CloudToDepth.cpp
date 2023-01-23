#include <iostream>
#include <chrono>
#include <cmath>
#include <string>
#include <functional>
#include <cstdlib>
#include <memory>
#include <cassert>

#include "CloudToDepth/CloudToDepth.hpp"

namespace perception {

using std::placeholders::_1;

CloudToDepth::CloudToDepth(const rclcpp::NodeOptions & options)
 : Node("cloud_to_depth", options) {
  source_topic = declare_parameter<std::string>("source_topic", "/point_cloud");
  dest_topic = declare_parameter<std::string>("dest_topic", "/depth_image");
  as_float = declare_parameter<bool>("as_float", false);
  
  publisher_ = create_publisher<sensor_msgs::msg::Image>(dest_topic, 10);

  subscription_ = create_subscription<Adapter>(
    source_topic, 1, 
    std::bind(&CloudToDepth::callback_pointcloud, this, std::placeholders::_1));
}

void CloudToDepth::callback_pointcloud(const std::shared_ptr<const StampedPointCloud2> &msg) {
  std::visit([&, header = msg->header](auto&& cloud) {this->process_message(header, cloud);}, msg->cloud);
} 

template <typename PointT>
void CloudToDepth::process_message(const std_msgs::msg::Header &header, const pcl::PointCloud<PointT> &cloud) {
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  
  
  if (cloud.height == 1) {
    RCLCPP_WARN(get_logger(), "Can't extract depth image from unstructured cloud.");
    return;
  }

  if (as_float){

    cv::Mat image(cloud.height, cloud.width, CV_32FC1);

    auto cp = cloud.points.begin();
    auto ip = image.begin<float>();
    for (; cp != cloud.points.end(); ++cp, ++ip) {
      (*ip) = cp->z;
    }
    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "32FC1", image).toImageMsg();
    publisher_->publish(*msg);

  } else {
    
    cv::Mat image(cloud.height, cloud.width, CV_16UC1);

    auto cp = cloud.points.begin();
    auto ip = image.begin<uint16_t>();
    for (; cp != cloud.points.end(); ++cp, ++ip) {
      (*ip) = std::isfinite(cp->z) ? cp->z * 1000 : 0; // See REP 118
    }

    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "16UC1", image).toImageMsg();
    publisher_->publish(*msg);
  }

  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  RCLCPP_INFO(get_logger(), "Time difference = %ld [ms]", std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());
}

}