/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node
 */

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include "src/CYdLidar.h"
#include <math.h>
#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include <vector>
#include <string>
#include <signal.h>
#include <limits>

#define ROS2Version "1.0.1"

int main(int argc, char *argv[]) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");

  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Current ROS Driver Version: %s", ROS2Version);

  CYdLidar laser;

  // Declare and retrieve parameters
  std::string port = "/dev/ttyUSB0";
  node->declare_parameter<std::string>("port", port);
  node->get_parameter("port", port);
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());

  std::string ignore_array = "";
  node->declare_parameter<std::string>("ignore_array", ignore_array);
  node->get_parameter("ignore_array", ignore_array);
  laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(), ignore_array.size());

  std::string frame_id = "laser_frame";
  node->declare_parameter<std::string>("frame_id", frame_id);
  node->get_parameter("frame_id", frame_id);

  int baudrate = 512000;
  node->declare_parameter<int>("baudrate", baudrate);
  node->get_parameter("baudrate", baudrate);
  laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));

  int lidar_type = TYPE_TRIANGLE;
  node->declare_parameter<int>("lidar_type", lidar_type);
  node->get_parameter("lidar_type", lidar_type);
  laser.setlidaropt(LidarPropLidarType, &lidar_type, sizeof(int));

  int device_type = YDLIDAR_TYPE_SERIAL;
  node->declare_parameter<int>("device_type", device_type);
  node->get_parameter("device_type", device_type);
  laser.setlidaropt(LidarPropDeviceType, &device_type, sizeof(int));

  int sample_rate = 9;
  node->declare_parameter<int>("sample_rate", sample_rate);
  node->get_parameter("sample_rate", sample_rate);
  laser.setlidaropt(LidarPropSampleRate, &sample_rate, sizeof(int));

  int abnormal_check_count = 4;
  node->declare_parameter<int>("abnormal_check_count", abnormal_check_count);
  node->get_parameter("abnormal_check_count", abnormal_check_count);
  laser.setlidaropt(LidarPropAbnormalCheckCount, &abnormal_check_count, sizeof(int));

  // Boolean parameters
  bool fixed_resolution = false;
  node->declare_parameter<bool>("fixed_resolution", fixed_resolution);
  node->get_parameter("fixed_resolution", fixed_resolution);
  laser.setlidaropt(LidarPropFixedResolution, &fixed_resolution, sizeof(bool));

  bool reversion = true;
  node->declare_parameter<bool>("reversion", reversion);
  node->get_parameter("reversion", reversion);
  laser.setlidaropt(LidarPropReversion, &reversion, sizeof(bool));

  bool inverted = true;
  node->declare_parameter<bool>("inverted", inverted);
  node->get_parameter("inverted", inverted);
  laser.setlidaropt(LidarPropInverted, &inverted, sizeof(bool));

  bool auto_reconnect = true;
  node->declare_parameter<bool>("auto_reconnect", auto_reconnect);
  node->get_parameter("auto_reconnect", auto_reconnect);
  laser.setlidaropt(LidarPropAutoReconnect, &auto_reconnect, sizeof(bool));

  bool is_single_channel = false;
  node->declare_parameter<bool>("isSingleChannel", is_single_channel);
  node->get_parameter("isSingleChannel", is_single_channel);
  laser.setlidaropt(LidarPropSingleChannel, &is_single_channel, sizeof(bool));

  bool intensity = false;
  node->declare_parameter<bool>("intensity", intensity);
  node->get_parameter("intensity", intensity);
  laser.setlidaropt(LidarPropIntenstiy, &intensity, sizeof(bool));

  bool support_motor_dtr = false;
  node->declare_parameter<bool>("support_motor_dtr", support_motor_dtr);
  node->get_parameter("support_motor_dtr", support_motor_dtr);
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &support_motor_dtr, sizeof(bool));

  // Float parameters
  float angle_max = 180.0f;
  node->declare_parameter<float>("angle_max", angle_max);
  node->get_parameter("angle_max", angle_max);
  laser.setlidaropt(LidarPropMaxAngle, &angle_max, sizeof(float));

  float angle_min = -180.0f;
  node->declare_parameter<float>("angle_min", angle_min);
  node->get_parameter("angle_min", angle_min);
  laser.setlidaropt(LidarPropMinAngle, &angle_min, sizeof(float));

  float range_max = 64.0f;
  node->declare_parameter<float>("range_max", range_max);
  node->get_parameter("range_max", range_max);
  laser.setlidaropt(LidarPropMaxRange, &range_max, sizeof(float));

  float range_min = 0.1f;
  node->declare_parameter<float>("range_min", range_min);
  node->get_parameter("range_min", range_min);
  laser.setlidaropt(LidarPropMinRange, &range_min, sizeof(float));

  float frequency = 10.0f;
  node->declare_parameter<float>("frequency", frequency);
  node->get_parameter("frequency", frequency);
  laser.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));

  bool invalid_range_is_inf = false;
  node->declare_parameter<bool>("invalid_range_is_inf", invalid_range_is_inf);
  node->get_parameter("invalid_range_is_inf", invalid_range_is_inf);

  // Publisher for LaserScan with updated QoS settings
  rclcpp::QoS qos(rclcpp::KeepLast(10));  // Keep last 10 messages in the buffer
  qos.reliable();  // Ensure reliable delivery of messages
  qos.durability_volatile();  // Volatile durability, meaning no retention of messages after disconnect

  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", qos);

  // Services to start and stop scan
  auto stop_scan_service = [&laser](const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                                    const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
                                    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
    laser.turnOff();
  };

  auto stop_service = node->create_service<std_srvs::srv::Empty>("stop_scan", stop_scan_service);

  auto start_scan_service = [&laser](const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                                     const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
                                     std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
    laser.turnOn();
  };

  auto start_service = node->create_service<std_srvs::srv::Empty>("start_scan", start_scan_service);

  bool ret = laser.initialize();
  if (ret) {
    ret = laser.turnOn();
  } else {
    RCLCPP_ERROR(node->get_logger(), "%s\n", laser.DescribeError());
  }

  rclcpp::WallRate loop_rate(20);
  while (ret && rclcpp::ok()) {
    sensor_msgs::msg::LaserScan scan_msg;
    LaserScan scan;

    if (laser.doProcessSimple(scan)) {
      // Fix timestamp issues
      auto now = node->get_clock()->now();
      scan_msg.header.stamp = now;

      scan_msg.header.frame_id = frame_id;
      scan_msg.angle_min = scan.config.min_angle;
      scan_msg.angle_max = scan.config.max_angle;
      scan_msg.angle_increment = scan.config.angle_increment;
      scan_msg.scan_time = scan.config.scan_time;
      scan_msg.time_increment = scan.config.time_increment;
      scan_msg.range_min = scan.config.min_range;
      scan_msg.range_max = scan.config.max_range;

      int size = (scan.config.max_angle - scan.config.min_angle) / scan.config.angle_increment + 1;
      scan_msg.ranges.resize(size);
      scan_msg.intensities.resize(size);
      for (size_t i = 0; i < scan.points.size(); i++) {
        int index = std::ceil((scan.points[i].angle - scan.config.min_angle) / scan.config.angle_increment);
        if (index >= 0 && index < size) {
          // Apply range filtering based on configuration
          scan_msg.ranges[index] = std::isfinite(scan.points[i].range) ? scan.points[i].range : std::numeric_limits<float>::infinity();
          scan_msg.intensities[index] = scan.points[i].intensity;
        }
      }

      laser_pub->publish(scan_msg);
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to get scan");
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Now YDLIDAR is stopping .......");
  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();

  return 0;
}
