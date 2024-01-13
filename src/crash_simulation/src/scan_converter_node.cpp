#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanConverter : public rclcpp::Node
{
public:
  ScanConverter()
      : Node("scan_converter")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(), std::bind(&ScanConverter::topic_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/laser_data", 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Total data count: '%zu'", msg->ranges.size());
    RCLCPP_INFO(this->get_logger(), "-------------------------------------------------------------");

    size_t row = 0;
    std::ostringstream oss;
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
      oss << "[" << msg->ranges[i] << "] ";
      // 10개 단위로 줄바꿈
      if ((i+1) % 10 == 0)
      {
        oss << std::endl;
      }
    }

    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    RCLCPP_INFO(this->get_logger(), "-------------------------------------------------------------");

    // 변환된 데이터 publish
    publisher_->publish(*msg);
  }
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanConverter>());
  rclcpp::shutdown();
  return 0;
}
