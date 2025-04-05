#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include "rclcpp/qos.hpp"  // Ensure correct inclusion

// Define a macro to convert radians to degrees
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

// Callback function to process received LaserScan data
static void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  if (!scan) {
    RCLCPP_ERROR(rclcpp::get_logger("YDLIDAR_Client"), "Received null scan data!");
    return;
  }

  int count = static_cast<int>(scan->scan_time / scan->time_increment);
  RCLCPP_INFO(rclcpp::get_logger("YDLIDAR_Client"), 
              "Received laser scan data from frame [%s], [%d] points detected.", 
              scan->header.frame_id.c_str(), count);

  RCLCPP_INFO(rclcpp::get_logger("YDLIDAR_Client"), 
              "Angle range: [%f, %f] degrees", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

  // Iterate through scan points and log angle-distance pairs
  for (int i = 0; i < count; ++i) {
    float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    float distance = scan->ranges[i];

    // Check for invalid range values (e.g., INF or NaN)
    if (std::isinf(distance) || std::isnan(distance)) {
      RCLCPP_WARN(rclcpp::get_logger("YDLIDAR_Client"), 
                  "Angle: [%f degrees], Invalid range value detected.", degree);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("YDLIDAR_Client"), 
                  "Angle: [%f degrees], Distance: [%f meters]", degree, distance);
    }
  }
}

int main(int argc, char **argv) {
  // Initialize the ROS 2 environment
  rclcpp::init(argc, argv);

  // Create a Node object
  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_client");

  // Define QoS settings for the subscriber to match system default policy
  rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();  // Set to default QoS

  // Subscribe to the "scan" topic with the specified QoS settings
  auto lidar_subscription = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", qos_settings, scanCallback);

  // Spin the Node to process incoming data
  RCLCPP_INFO(node->get_logger(), "YDLIDAR ROS 2 Client Node has started.");
  rclcpp::spin(node);

  // Shutdown the ROS 2 environment
  RCLCPP_INFO(node->get_logger(), "Shutting down YDLIDAR ROS 2 Client Node.");
  rclcpp::shutdown();

  return 0;
}
