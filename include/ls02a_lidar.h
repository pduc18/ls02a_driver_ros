#ifndef LS02A_LIDAR_H
#define LS02A_LIDAR_H

#include <sensor_msgs/msg/laser_scan.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <string>
#include <memory>

namespace ls02a_lidar_driver {
class ls02a_lidar
{
public:
  uint16_t rpms;

  /**
   * @brief Constructs a new XV11Laser attached to the given serial port
   * @param port The string for the serial port device to attempt to connect to, e.g. "/dev/ttyUSB0"
   * @param baud_rate The baud rate to open the serial port at.
   * @param io Boost ASIO IO Service to use when creating the serial port object
   */
  ls02a_lidar(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io);

  /**
   * @brief Default destructor
   */
  ~ls02a_lidar();

  /**
   * @brief Poll the laser to get a new scan. Blocks until a complete new scan is received or close is called.
   * @param scan LaserScan message pointer to fill in with the scan. The caller is responsible for filling in the ROS timestamp and frame_id
   */
  void poll(sensor_msgs::msg::LaserScan::SharedPtr scan, int version_num);

  /**
   * @brief Close the driver down and prevent the polling loop from advancing
   */
  void close();

private:
  std::string port_; ///< @brief The serial port the driver is attached to
  uint32_t baud_rate_; ///< @brief The baud rate for the serial connection
  int version_num;

  bool shutting_down_; ///< @brief Flag for whether the driver is supposed to be shutting down or not
  boost::asio::serial_port serial_; ///< @brief Actual serial port object for reading/writing to the ls02a lidar Scanner
  uint16_t motor_speed_; ///< @brief current motor speed as reported by the ls02a lidar.

  /**
   * @brief checkSum
   * @param p_byte
   * @return
   */
  uint16_t checkSum(const uint8_t *p_byte);
};

} // namespace ls02a_lidar_driver

#endif // ls02a_LIDAR_H