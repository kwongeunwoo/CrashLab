#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class RpmSubscriber : public rclcpp::Node
{
public:
  RpmSubscriber() : Node("rpm_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "rpm", 10, std::bind(&RpmSubscriber::topic_callback, this, std::placeholders::_1));
    
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("pid_check", 10);
  }

private:
  void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if(msg->data.size() != 4)
    {
      RCLCPP_ERROR(this->get_logger(), "Received data size is not correct for rpm");
      return;
    }
    double rpm_value1 = msg->data[0];
    double rpm_value2 = msg->data[1];
    double target_rpm1 = msg->data[2];
    double target_rpm2 = msg->data[3];
    
    RCLCPP_INFO(this->get_logger(), "Received rpm values: rpm_value1: '%f', rpm_value2: '%f', target_rpm1: '%f', target_rpm2: '%f'", 
        rpm_value1, rpm_value2, target_rpm1, target_rpm2);

    // Create and publish a new message
    std_msgs::msg::Float64MultiArray pub_msg;
    pub_msg.data = {rpm_value1, rpm_value2, target_rpm1, target_rpm2};
    publisher_->publish(pub_msg);
  }

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_; // Add publisher
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RpmSubscriber>());
  rclcpp::shutdown();
  return 0;
}
