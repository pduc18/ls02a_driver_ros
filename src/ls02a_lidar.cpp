#include "ls02a_lidar.h"
#include <iostream>

using namespace std;
namespace ls02a_lidar_driver {
ls02a_lidar::ls02a_lidar(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io)
  : port_(port), baud_rate_(baud_rate), shutting_down_(false), serial_(io, port_)
{
  try {
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    std::cout << "Serial port opened: " << port_ << " at " << baud_rate_ << " baud" << std::endl;
  } catch (...) {
    std::cerr << "Failed to open serial port" << std::endl;
    throw;
  }
}

ls02a_lidar::~ls02a_lidar()
{
  serial_.close();
}

void ls02a_lidar::poll(sensor_msgs::msg::LaserScan::SharedPtr scan, int version_num)
{
  uint8_t start_count = 0;
  bool got_scan = false;

  if (version_num == 1) {
    boost::array<uint8_t, 660> raw_bytes;
    uint8_t good_sets = 0;
    uint32_t motor_speed = 0;
    rpms = 0;
    int index;
    while (!shutting_down_ && !got_scan) {
      boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[start_count], 1));
      if (start_count == 0) {
        if (raw_bytes[start_count] == 0xFA) {
          start_count = 1;
          std::cout << "Found start byte" << std::endl;
        }
      } else if (start_count == 1) {
        if (raw_bytes[start_count] == 0xA0) {
          start_count = 0;
          std::cout << "Found sync byte" << std::endl;
          got_scan = true;
          boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[2], 658));
          std::cout << "Read " << 658 << " bytes" << std::endl;

          scan->angle_min = 46.0 * M_PI / 180.0;
          scan->angle_max = -43.0 * M_PI / 180.0;
          scan->angle_increment = -(2.0 * M_PI / 360.0);
          scan->range_min = 0.15;
          scan->range_max = 6.0;
          scan->ranges.resize(120);
          scan->intensities.resize(120);

          for (uint16_t i = 0; i < raw_bytes.size(); i += 22) {
            if (raw_bytes[i] == 0xFA && raw_bytes[i + 1] == (0xA0 + i / 22)) {
              good_sets++;
              motor_speed += (raw_bytes[i + 3] << 8) + raw_bytes[i + 2];
              rpms = (raw_bytes[i + 3] << 8 | raw_bytes[i + 2]) / 64;

              for (uint16_t j = i + 4; j < i + 20; j += 4) {
                index = (4 * i) / 22 + (j - 4 - i) / 4;
                uint8_t byte0 = raw_bytes[j];
                uint8_t byte1 = raw_bytes[j + 1];
                uint8_t byte2 = raw_bytes[j + 2];
                uint16_t range = ((byte1 & 0x3F) << 8) + byte0;
                uint16_t intensity_width = byte2;

                scan->ranges[index] = range / 1000.0;
                scan->intensities[index] = 100 * intensity_width;
              }
            }
          }
          scan->time_increment = motor_speed / good_sets / 1e8;
          if (scan->ranges.empty()) {
            std::cerr << "No valid data in scan->ranges" << std::endl;
          }
        }
      }
    }
  } else if (version_num == 2) {
    boost::array<uint8_t, 946> raw_bytes;
    uint8_t good_sets = 0;
    uint32_t motor_speed = 0;
    rpms = 0;
    int index;
    while (!shutting_down_ && !got_scan) {
      boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[start_count], 1));
      if (start_count == 0) {
        if (raw_bytes[start_count] == 0xFA) {
          start_count = 1;
          std::cout << "Found start byte" << std::endl;
        }
      } else if (start_count == 1) {
        if (raw_bytes[start_count] == 0xA0) {
          start_count = 0;
          std::cout << "Found sync byte" << std::endl;
          got_scan = true;
          boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[2], 944));
          std::cout << "Read " << 944 << " bytes" << std::endl;

          scan->angle_min = 43.0 * M_PI / 180.0;
          scan->angle_max = -42.5 * M_PI / 180.0;
          scan->angle_increment = -(1.0 * M_PI / 360.0);
          scan->range_min = 0.15;
          scan->range_max = 6.0;
          scan->ranges.resize(172);
          scan->intensities.resize(172);

          for (uint16_t i = 0; i < raw_bytes.size(); i += 22) {
            if (raw_bytes[i] == 0xFA && raw_bytes[i + 1] == (0xA0 + i / 22)) {
              good_sets++;
              motor_speed += (raw_bytes[i + 3] << 8) + raw_bytes[i + 2];
              rpms = (raw_bytes[i + 3] << 8 | raw_bytes[i + 2]) / 64;

              for (uint16_t j = i + 4; j < i + 20; j += 4) {
                index = (4 * i) / 22 + (j - 4 - i) / 4;
                uint8_t byte0 = raw_bytes[j];
                uint8_t byte1 = raw_bytes[j + 1];
                uint8_t byte2 = raw_bytes[j + 2];
                uint16_t range = ((byte1 & 0x3F) << 8) + byte0;
                uint16_t intensity_width = byte2;

                scan->ranges[index] = range / 1000.0;
                scan->intensities[index] = 100 * intensity_width;
              }
            }
          }
          scan->time_increment = motor_speed / good_sets / 1e8;
          if (scan->ranges.empty()) {
            std::cerr << "No valid data in scan->ranges" << std::endl;
          }
        }
      }
    }
  } else if (version_num == 0) {
    boost::array<uint8_t, 1980> raw_bytes;
    uint8_t good_sets = 0;
    uint32_t motor_speed = 0;
    rpms = 0;
    int index;
    while (!shutting_down_ && !got_scan) {
      boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[start_count], 1));
      if (start_count == 0) {
        if (raw_bytes[start_count] == 0xFA) {
          start_count = 1;
          std::cout << "Found start byte" << std::endl;
        }
      } else if (start_count == 1) {
        if (raw_bytes[start_count] == 0xA0) {
          start_count = 0;
          std::cout << "Found sync byte" << std::endl;
          got_scan = true;
          boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[2], 1978));
          std::cout << "Read " << 1978 << " bytes" << std::endl;

          scan->angle_min = 0 * M_PI / 180.0;
          scan->angle_max = 360.0 * M_PI / 180.0;
          scan->angle_increment = (2.0 * M_PI / 360.0);
          scan->range_min = 0.15;
          scan->range_max = 10.0;
          scan->ranges.resize(360);
          scan->intensities.resize(360);

          for (uint16_t i = 0; i < raw_bytes.size(); i += 22) {
            if (raw_bytes[i] == 0xFA && raw_bytes[i + 1] == (0xA0 + i / 22)) {
              good_sets++;
              motor_speed += (raw_bytes[i + 3] << 8) + raw_bytes[i + 2];
              rpms = (raw_bytes[i + 3] << 8 | raw_bytes[i + 2]) / 64;

              for (uint16_t j = i + 4; j < i + 20; j += 4) {
                index = (4 * i) / 22 + (j - 4 - i) / 4;
                uint8_t byte0 = raw_bytes[j];
                uint8_t byte1 = raw_bytes[j + 1];
                uint8_t byte2 = raw_bytes[j + 2];
                uint16_t range = ((byte1 & 0x3F) << 8) + byte0;
                uint16_t intensity_width = byte2;

                if (range < 15)
                  scan->ranges[359 - index] = std::numeric_limits<float>::infinity();
                else
                  scan->ranges[index] = range / 1000.0;
                scan->intensities[index] = 100 * intensity_width;
              }
            }
          }
          scan->time_increment = motor_speed / good_sets / 1e8;
          if (scan->ranges.empty()) {
            std::cerr << "No valid data in scan->ranges" << std::endl;
          }
        }
      }
    }
  }
}

void ls02a_lidar::close()
{
  shutting_down_ = true;
}
} // namespace n301_lidar_driver