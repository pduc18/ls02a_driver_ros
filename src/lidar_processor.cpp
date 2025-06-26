#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <boost/asio.hpp>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <mutex>
#include <thread>
#include <queue>
#include <condition_variable>

class LidarProcessor : public rclcpp::Node
{
public:
  LidarProcessor()
  : Node("lidar_processor"), io_(), serial_(io_), exit_flag_(false)
  {
    // UART parameters
    this->declare_parameter<std::string>("uart_port", "/dev/ttyUSB1");
    this->declare_parameter<int>("uart_baud_rate", 115200);
    std::string uart_port = this->get_parameter("uart_port").as_string();
    int uart_baud_rate = this->get_parameter("uart_baud_rate").as_int();

    // Open UART
    try {
      serial_.open(uart_port);
      serial_.set_option(boost::asio::serial_port_base::baud_rate(uart_baud_rate));
      RCLCPP_INFO(this->get_logger(), "Opened UART port: %s at %d baud", uart_port.c_str(), uart_baud_rate);
    } catch (const boost::system::system_error& ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open UART port: %s, Error: %s", uart_port.c_str(), ex.what());
      rclcpp::shutdown();
      return;
    }

    // Sub to /scan
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&LidarProcessor::scan_callback, this, std::placeholders::_1));

    // Start UART thread
    uart_thread_ = std::thread(&LidarProcessor::uart_writer_thread, this);

    RCLCPP_INFO(this->get_logger(), "LidarProcessor started.");
  }

  ~LidarProcessor()
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      exit_flag_ = true;
    }
    queue_cv_.notify_all();
    if (uart_thread_.joinable()) {
      uart_thread_.join();
    }

    if (serial_.is_open()) {
      serial_.close();
      RCLCPP_INFO(this->get_logger(), "UART port closed");
    }
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    float min_range = msg->range_max;
    size_t min_index = 0;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float r = msg->ranges[i];
      if (r >= msg->range_min && r <= msg->range_max && r < min_range) {
        min_range = r;
        min_index = i;
      }
    }

    if (min_range == msg->range_max) return;

    float min_angle = (msg->angle_min + min_index * msg->angle_increment) * 180.0 / M_PI;

    std::stringstream ss;
    ss << "#" << std::fixed << std::setprecision(2)
       << min_range << "," << min_angle << "\r\n";

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      message_queue_.push(ss.str());
    }
    queue_cv_.notify_one();
  }

  void uart_writer_thread()
  {
    while (rclcpp::ok()) {
      std::string message;

      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this]() {
          return !message_queue_.empty() || exit_flag_;
        });

        if (exit_flag_ && message_queue_.empty()) break;

        message = message_queue_.front();
        message_queue_.pop();
      }

      try {
        boost::asio::write(serial_, boost::asio::buffer(message));
        RCLCPP_INFO(this->get_logger(), "Sent UART: %s", message.c_str());
      } catch (const boost::system::system_error& ex) {
        RCLCPP_ERROR(this->get_logger(), "UART write error: %s", ex.what());
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  boost::asio::io_service io_;
  boost::asio::serial_port serial_;

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::queue<std::string> message_queue_;
  std::thread uart_thread_;
  bool exit_flag_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarProcessor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
