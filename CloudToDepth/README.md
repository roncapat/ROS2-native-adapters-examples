# ROS2 Point Cloud to Depth Map

This package converts a structured Point Cloud to a Depth Map (raster 2D image where pixel values are the Z depths).
It is optimized for subscriptions to publishers using https://github.com/roncapat/ros2_native_adapters, leveraging zero-copy capabilities of rclcpp::components feature and providing exchange of native PCL structures.