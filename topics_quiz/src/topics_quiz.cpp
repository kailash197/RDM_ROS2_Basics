#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

class TopicsQuiz : public rclcpp::Node {
public:
  TopicsQuiz() : Node("topics_quiz_node") {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&TopicsQuiz::laser_scan_callback, this, _1));
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
        50ms, std::bind(&TopicsQuiz::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = geometry_msgs::msg::Twist();
    if (not front_obstacle) {
      message.linear.x = 0.5;
      message.angular.z = 0.0;

    } else if (left_obstacle) {
      message.linear.x = 0.2;
      message.angular.z = -0.5;
    } else if (right_obstacle) {
      message.linear.x = 0.2;
      message.angular.z = -0.5;
    } else {
      message.linear.x = 0;
      message.angular.z = 0.5;
    }

    publisher_->publish(message);
  }

  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // int length = 720;
    float front_distance = 0.0, left_distance = 0.0, right_distance = 0.0;

    for (int i = 0; i < 15; i++) {
      right_distance += msg->ranges[i];
      left_distance += msg->ranges[720 - 1 - i];
      front_distance += msg->ranges[(720 / 2 - 7) + i];
    }
    front_obstacle = (front_distance / 15) < 1.0;
    left_obstacle = (left_distance / 15) < 1.0;
    right_obstacle = (right_obstacle / 15) < 1.0;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  bool left_obstacle = false;
  bool right_obstacle = false;
  bool front_obstacle = false;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicsQuiz>());
  rclcpp::shutdown();
  return 0;
}