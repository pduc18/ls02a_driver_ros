#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <boost/asio.hpp>
#include <fstream>
#include <vector>
#include <csignal>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <rcpputils/filesystem_helper.hpp>
#include "ls02a_lidar.h"

volatile sig_atomic_t g_signal_status = 0;

void signal_handler(int signal) {
  g_signal_status = signal;
}

class LidarNode : public rclcpp::Node
{
public:
  LidarNode() : Node("line_laser")
  {
    std::signal(SIGINT, signal_handler);

    // Declare and get parameters
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<std::string>("frame_id", "base_laser_link");
    this->declare_parameter<int>("version_num", 1);
    this->declare_parameter<std::string>("log_dir", std::string(std::getenv("HOME")) + "/ros2_ws/log");

    port_ = this->get_parameter("port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();
    version_num_ = this->get_parameter("version_num").as_int();
    std::string log_dir = this->get_parameter("log_dir").as_string();

    // Generate unique CSV filename with timestamp in workspace log directory
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S");
    // Create log directory if it doesn't exist
    if (!rcpputils::fs::create_directories(log_dir)) {
      RCLCPP_WARN(this->get_logger(), "Failed to create log directory: %s", log_dir.c_str());
    }
    csv_file_ = log_dir + "/lidar_data_" + ss.str() + ".csv";

    RCLCPP_INFO(this->get_logger(), "Starting LidarNode with port: %s, baud: %d, frame_id: %s, version: %d",
                port_.c_str(), baud_rate_, frame_id_.c_str(), version_num_);

    // Create publishers
    laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 1000);
    motor_pub_ = this->create_publisher<std_msgs::msg::UInt16>("rpms", 1000);
    RCLCPP_INFO(this->get_logger(), "Publishers created for /scan and /rpms");

    // Open CSV file
    RCLCPP_INFO(this->get_logger(), "Attempting to open CSV file: %s", csv_file_.c_str());
    csv_file_stream_.open(csv_file_, std::ios::out);
    if (!csv_file_stream_.is_open() || !csv_file_stream_.good()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s. Continuing without CSV logging.", csv_file_.c_str());
      csv_file_ = "";
    } else {
      csv_file_stream_ << "Angle(deg),Range(m),Intensity\n";
      csv_file_stream_.flush();
      RCLCPP_INFO(this->get_logger(), "CSV file opened and header written");
    }

    // Initialize LiDAR
    try {
      laser_ = std::make_unique<ls02a_lidar_driver::ls02a_lidar>(port_, baud_rate_, io_);
      RCLCPP_INFO(this->get_logger(), "LiDAR initialized successfully");
      
      switch (version_num_) {
        case 0:
          RCLCPP_INFO(this->get_logger(), "It is ls02a TOF with 360 deg FOV");
          break;
        case 1:
          RCLCPP_INFO(this->get_logger(), "It is line laser with 1 deg resolution");
          break;
        case 2:
          RCLCPP_INFO(this->get_logger(), "It is line laser with 0.5 deg resolution");
          break;
        default:
          RCLCPP_INFO(this->get_logger(), "It is default setting that line laser with 1 deg resolution");
      }

      // Start polling
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&LidarNode::poll_lidar, this));
      RCLCPP_INFO(this->get_logger(), "Polling timer started");
    }
    catch (const boost::system::system_error& ex) {
      RCLCPP_ERROR(this->get_logger(), "Error instantiating laser object. Port: %s, Baud: %d, Error: %s",
                   port_.c_str(), baud_rate_, ex.what());
      csv_file_stream_.close();
      rclcpp::shutdown();
    }
  }

  ~LidarNode()
  {
    if (laser_) {
      laser_->close();
      RCLCPP_INFO(this->get_logger(), "LiDAR closed");
    }
    if (csv_file_stream_.is_open()) {
      csv_file_stream_.close();
      RCLCPP_INFO(this->get_logger(), "CSV file closed");
    }
  }

private:
  void poll_lidar()
  {
    if (g_signal_status == SIGINT) {
      RCLCPP_INFO(this->get_logger(), "Received SIGINT, stopping polling");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Polling LiDAR data");
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    scan->header.frame_id = frame_id_;
    scan->header.stamp = this->now();

    try {
      laser_->poll(scan, version_num_);
    } catch (const boost::system::system_error& ex) {
      RCLCPP_ERROR(this->get_logger(), "Error polling LiDAR: %s", ex.what());
      return;
    }

    std_msgs::msg::UInt16 rpms;
    rpms.data = laser_->rpms;

    if (scan->ranges.empty()) {
      RCLCPP_WARN(this->get_logger(), "No data in scan->ranges");
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Publishing scan with %zu ranges", scan->ranges.size());
      laser_pub_->publish(*scan);
      motor_pub_->publish(rpms);

      // Write to CSV
      if (!csv_file_.empty() && csv_file_stream_.is_open() && csv_file_stream_.good()) {
        RCLCPP_DEBUG(this->get_logger(), "Writing %zu points to CSV", scan->ranges.size());
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
          float angle = scan->angle_min + i * scan->angle_increment;
          angle = angle * 180.0 / M_PI;
          csv_file_stream_ << angle << "," << scan->ranges[i] << "," << scan->intensities[i] << "\n";
        }
        csv_file_stream_.flush();
        if (!csv_file_stream_.good()) {
          RCLCPP_ERROR(this->get_logger(), "Failed to write data to CSV");
          csv_file_stream_.close();
          csv_file_ = "";
        }
        RCLCPP_DEBUG(this->get_logger(), "Wrote data to CSV");
      }
    }
  }

  std::string port_;
  int baud_rate_;
  std::string frame_id_;
  int version_num_;
  std::string csv_file_;
  std::ofstream csv_file_stream_;
  boost::asio::io_service io_;
  std::unique_ptr<ls02a_lidar_driver::ls02a_lidar> laser_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr motor_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
