#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include <std_msgs/msg/float32.hpp>
#include "std_msgs/msg/int8.hpp"

double error_ = 0.0;
bool turn_flag = true;
bool goal_pub_flag = true;
bool arrive_flag = false;

class NavigateControllerNode : public rclcpp::Node
{
public:
  NavigateControllerNode()
      : Node("navigate_controller_node"),
        goal_handle_future()
  {
    // 퍼블리셔 및 서브스크라이버 생성
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_control", 10);
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&NavigateControllerNode::cmd_vel_callback, this, std::placeholders::_1));
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
    goal_status_publisher_ = this->create_publisher<std_msgs::msg::Bool>("goal_status", 10);
    goal_trigger_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/behaviorFromGPT", 10, std::bind(&NavigateControllerNode::goal_trigger_callback, this, std::placeholders::_1));
    motion_command_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
        "distance_f_person", 10, std::bind(&NavigateControllerNode::motion_command_callback, this, std::placeholders::_1));

    // 액션 클라이언트 생성
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this, "/navigate_to_pose");

    pose_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SensorDataQoS(), std::bind(&NavigateControllerNode::pose_callback, this, std::placeholders::_1));

    goal_success_publisher_ = this->create_publisher<std_msgs::msg::String>("nowPosition", 10);

    HNCheck_subscriber_ = this->create_subscription<std_msgs::msg::Int8>(
        "HNCheck", 10, std::bind(&NavigateControllerNode::HNCheck_callback, this, std::placeholders::_1));

    // 초기 위치 설정 및 퍼블리싱
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = this->now();
    initial_pose.pose.pose.position.x = 0.0;
    initial_pose.pose.pose.position.y = 0.0;
    initial_pose.pose.pose.position.z = 0.0;
    initial_pose.pose.pose.orientation.x = 0.0;
    initial_pose.pose.pose.orientation.y = 0.0;
    initial_pose.pose.pose.orientation.z = 0.0;
    initial_pose.pose.pose.orientation.w = 1.0;

    pose_publisher_->publish(initial_pose);

    compare_locate_msg = std::make_shared<std_msgs::msg::String>();
    last_HNCheck_msg = std::make_shared<std_msgs::msg::Int8>();
    mode_compare.data = 2;
  }

private:
  // cmd_vel 메시지 콜백 함수
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (last_HNCheck_msg->data == mode_compare.data)
    {
      last_cmd_vel_ = msg;

      if (last_locate_msg->data == "stop" && goal_handle_future.valid())
      {
        auto goal_handle_ptr = goal_handle_future.get();
        if (goal_handle_ptr && goal_handle_ptr->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING)
        {
          auto future_cancel = action_client_->async_cancel_goal(goal_handle_ptr);
          goal_handle_future = std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr>();
          arrive_flag = true;
          compare_locate_msg = last_locate_msg;
          RCLCPP_INFO(this->get_logger(), "real stop"); // 로그 메시지 추가
        }
      }

      if (turn_flag != true)
      {
        if (last_cmd_vel_)
        {
          cmd_vel_publisher_->publish(*last_cmd_vel_);
        }
        else if (turn_flag != true)
        {
          geometry_msgs::msg::Twist stop_msg;
          stop_msg.linear.x = 0.0;
          stop_msg.linear.y = 0.0;
          stop_msg.linear.z = 0.0;
          stop_msg.angular.x = 0.0;
          stop_msg.angular.y = 0.0;
          stop_msg.angular.z = 0.0;
          cmd_vel_publisher_->publish(stop_msg);
        }
      }
    }
  }

  void HNCheck_callback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    last_HNCheck_msg = msg;
    if (last_HNCheck_msg->data != mode_compare.data)
    {
      turn_flag = true;
      goal_pub_flag = true;
      arrive_flag = false;
      compare_locate_msg = std::make_shared<std_msgs::msg::String>();
    }
  }

  // movtion_command 메시지 콜백 함수
  void motion_command_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (msg->data <= 2.0 && turn_flag != true)
    {
      if (last_cmd_vel_)
      {
        cmd_vel_publisher_->publish(*last_cmd_vel_);
      }
      else if (turn_flag != true)
      {
        geometry_msgs::msg::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.linear.y = 0.0;
        stop_msg.linear.z = 0.0;
        stop_msg.angular.x = 0.0;
        stop_msg.angular.y = 0.0;
        stop_msg.angular.z = 0.0;
        cmd_vel_publisher_->publish(stop_msg);
      }
    }

    else if (turn_flag != true)
    {
      geometry_msgs::msg::Twist stop_msg;
      stop_msg.linear.x = 0.0;
      stop_msg.linear.y = 0.0;
      stop_msg.linear.z = 0.0;
      stop_msg.angular.x = 0.0;
      stop_msg.angular.y = 0.0;
      stop_msg.angular.z = 0.0;
      cmd_vel_publisher_->publish(stop_msg);
    }
  }

  // 목표 위치 설정 요청 콜백 함수
  void goal_trigger_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    last_locate_msg = msg;

    if (last_locate_msg->data == "stop" && goal_handle_future.valid())
    {
      auto goal_handle_ptr = goal_handle_future.get();
      if (goal_handle_ptr && goal_handle_ptr->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING)
      {
        auto future_cancel = action_client_->async_cancel_goal(goal_handle_ptr);
        goal_handle_future = std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr>();
        arrive_flag = true;
        compare_locate_msg = last_locate_msg;
        RCLCPP_INFO(this->get_logger(), "real stop"); // 로그 메시지 추가
      }
    }
  }

  // 액션 서버로부터의 응답 콜백 함수
  void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
  {
    std_msgs::msg::Bool goal_status_msg;
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
      goal_status_msg.data = true;
      RCLCPP_INFO(this->get_logger(), "Arrived at the goal"); // 로그 메시지 추가
      compare_locate_msg = last_locate_msg;
      arrive_flag = true;
      break;
    default:
      goal_status_msg.data = false;
      if (last_locate_msg->data == "stop")
      {
        RCLCPP_ERROR(this->get_logger(), "who_stop_robot"); // 로그 메시지 추가
        compare_locate_msg = last_locate_msg;
        arrive_flag = true;
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to arrive at the goal"); // 로그 메시지 추가
        compare_locate_msg = last_locate_msg;
        arrive_flag = true;
      }

      break;
    }
    goal_status_publisher_->publish(goal_status_msg);
  }

  void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_ = msg->pose.pose;

    if (last_locate_msg && last_HNCheck_msg->data == mode_compare.data)
    {
      // 목표 위치 설정
      auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
      // 신발장
      if (last_locate_msg->data == "01")
      {
        goal_msg.pose.pose.position.x = 3.0;
        goal_msg.pose.pose.position.y = 1.0;
        // x축 기준 반시계방향 90도
        tf2::Quaternion q;
        q.setRPY(0, 0, 1.5708); // Roll, Pitch, Yaw (0, 0, 90 degrees)

        // tf2 쿼터니언을 geometry_msgs 타입으로 변환하여 대입
        goal_msg.pose.pose.orientation = tf2::toMsg(q);
      }
      // 옷가게
      else if (last_locate_msg->data == "02")
      {
        goal_msg.pose.pose.position.x = -2.0;
        goal_msg.pose.pose.position.y = 1.0;
        // x축 기준 시계방향 90도
        tf2::Quaternion q;
        q.setRPY(0, 0, -1.5708); // Roll, Pitch, Yaw (0, 0, -90 degrees)

        // tf2 쿼터니언을 geometry_msgs 타입으로 변환하여 대입
        goal_msg.pose.pose.orientation = tf2::toMsg(q);
      }
      // 책
      else if (last_locate_msg->data == "03")
      {
        goal_msg.pose.pose.position.x = 6.0;
        goal_msg.pose.pose.position.y = -6.0;
        // x축 기준 시계방향 90도
        tf2::Quaternion q;
        q.setRPY(0, 0, -1.5708); // Roll, Pitch, Yaw (0, 0, -90 degrees)

        // tf2 쿼터니언을 geometry_msgs 타입으로 변환하여 대입
        goal_msg.pose.pose.orientation = tf2::toMsg(q);
      }
      // 전자제품
      else if (last_locate_msg->data == "04")
      {
        goal_msg.pose.pose.position.x = 0.0;
        goal_msg.pose.pose.position.y = 0.0;
        // x축 기준 시계방향 180도
        tf2::Quaternion q;
        q.setRPY(0, 0, -3.14); // Roll, Pitch, Yaw (0, 0, -180 degrees)

        // tf2 쿼터니언을 geometry_msgs 타입으로 변환하여 대입
        goal_msg.pose.pose.orientation = tf2::toMsg(q);
      }
      else if (last_locate_msg->data == "stop")
      {
        // RCLCPP_INFO(this->get_logger(), "stop");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Invalid location");
        return;
      }

      // 목표 위치와 현재 위치 사이의 각도를 계산
      double target_angle = atan2(goal_msg.pose.pose.position.y - current_pose_.position.y, goal_msg.pose.pose.position.x - current_pose_.position.x);
      double current_angle = tf2::getYaw(current_pose_.orientation);

      // 현재 각도와 목표 각도 사이의 차이를 계산
      double error = target_angle - current_angle;
      error = atan2(sin(error), cos(error));
      double error_sign = std::signbit(error) ? -1 : 1;

      if (last_locate_msg->data != compare_locate_msg->data && arrive_flag == true)
      {
        turn_flag = true;
        goal_pub_flag = true;
        arrive_flag = false;
        RCLCPP_INFO(this->get_logger(), "11111");
      }

      // 5도를 라디안으로 변환하여 오차 범위 내에 있으면 회전하지 않음
      if (abs(error) >= 5.0 * M_PI / 180.0 && turn_flag == true && arrive_flag == false)
      {
        geometry_msgs::msg::Twist rotate_msg;
        rotate_msg.angular.z = error_sign * 0.65;

        // 회전 명령을 전송
        cmd_vel_publisher_->publish(rotate_msg);
        RCLCPP_INFO(this->get_logger(), "22222");
      }

      else if (last_locate_msg->data != compare_locate_msg->data && goal_pub_flag == true)
      {
        // 액션 서버로 Goal 전송 설정
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            std::bind(&NavigateControllerNode::result_callback, this, std::placeholders::_1);
        goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);

        turn_flag = false;
        goal_pub_flag = false;
        RCLCPP_INFO(this->get_logger(), "33333");
      }

      if (arrive_flag == true)
      {
        goal_success_publisher_->publish(*last_locate_msg);
      }
    }
  }


  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_status_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_trigger_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_success_publisher_;


  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr motion_command_subscription_;
  geometry_msgs::msg::Twist::SharedPtr last_cmd_vel_;
  geometry_msgs::msg::Pose current_pose_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscriber_;

  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr current_goal_handle_;
  std_msgs::msg::String::SharedPtr last_locate_msg;
  std_msgs::msg::String::SharedPtr compare_locate_msg;
  std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> goal_handle_future;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr HNCheck_subscriber_;
  std_msgs::msg::Int8::SharedPtr last_HNCheck_msg;
  std_msgs::msg::Int8 mode_compare;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigateControllerNode>());
  rclcpp::shutdown();
  return 0;
}