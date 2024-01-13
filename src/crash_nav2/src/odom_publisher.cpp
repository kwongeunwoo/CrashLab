#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <chrono>
using namespace std::chrono_literals;

class OdomPublisher : public rclcpp::Node
{
public:
  OdomPublisher()
  : Node("odom_publisher"), tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this)), x_(0.0)
  {
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS());
    timer_ = this->create_wall_timer(
      100ms, std::bind(&OdomPublisher::publish_odom, this));
  }

private:
  void publish_odom()
  {
    x_ += 0.01;  // 0.1m per second

    auto message = nav_msgs::msg::Odometry();
    message.header.stamp = this->now();
    message.header.frame_id = "odom";
    message.child_frame_id = "base_link";
    message.pose.pose.position.x = x_;
    message.pose.pose.position.y = 0.0;
    message.pose.pose.position.z = 0.0;
    message.twist.twist.linear.x = 0.1;
    message.twist.twist.linear.y = 0.0;
    message.twist.twist.linear.z = 0.0;

    odom_publisher_->publish(message);

    auto tf = geometry_msgs::msg::TransformStamped();
    tf.header.stamp = this->now();
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = x_;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(tf);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  double x_;  // add this line
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPublisher>());
  rclcpp::shutdown();
  return 0;
}
