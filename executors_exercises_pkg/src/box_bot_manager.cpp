#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/pose__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/visibility_control.hpp"
#include <functional>
#include <geometry_msgs/msg/point.h>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string.h>
using namespace std;

typedef geometry_msgs::msg::Point Point;

class BoxBotManager : public rclcpp::Node {
public:
  BoxBotManager(string bb1_topic, string bb2_topic, string bb3_topic,
                Point goal1, Point goal2, Point goal3, float time)
      : Node("Box_Bot_Manager_Node") {
    this->bb1_topic_name = bb1_topic;
    this->bb2_topic_name = bb2_topic;
    this->bb3_topic_name = bb3_topic;
    this->goal1 = goal1;
    this->goal2 = goal2;
    this->goal3 = goal3;
    RCLCPP_INFO(this->get_logger(), "INIT GOAL 1 >>>>>>>>>>>>>>>['%f','%f']",
                this->goal1.x, this->goal1.y);
    RCLCPP_INFO(this->get_logger(), "INIT GOAL 2 >>>>>>>>>>>>>>>['%f','%f']",
                this->goal2.x, this->goal2.y);
    RCLCPP_INFO(this->get_logger(), "INIT  GOAL 3 >>>>>>>>>>>>>>>['%f','%f']",
                this->goal3.x, this->goal3.y);
    this->wait_time = time;

    timer_callback_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    odom1_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    odom2_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    odom3_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = odom1_callback_group_;
    rclcpp::SubscriptionOptions options2;
    options2.callback_group = odom2_callback_group_;
    rclcpp::SubscriptionOptions options3;
    options3.callback_group = odom3_callback_group_;

    bb1_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        bb1_topic_name, 10,
        bind(&BoxBotManager::bb1_subscriber_callback, this,
             std::placeholders::_1),
        options1);
    bb2_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        bb2_topic_name, 10,
        bind(&BoxBotManager::bb2_subscriber_callback, this,
             std::placeholders::_1),
        options2);
    bb3_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        bb3_topic_name, 10,
        bind(&BoxBotManager::bb3_subscriber_callback, this,
             std::placeholders::_1),
        options3);

    timer_ = this->create_wall_timer(500ms,
                                     bind(&BoxBotManager::timer_callback, this),
                                     timer_callback_group);

    publisher1_ = this->create_publisher<std_msgs::msg::String>(
        "/reached_goal/box_bot_1", 1);
    publisher2_ = this->create_publisher<std_msgs::msg::String>(
        "/reached_goal/box_bot_2", 1);
    publisher3_ = this->create_publisher<std_msgs::msg::String>(
        "/reached_goal/box_bot_3", 1);
    RCLCPP_INFO(this->get_logger(), "BoxBotManager Object Created.");
  }

  void bb1_subscriber_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    this->box_bot_1_reached_goal =
        check_reached_goal(this->goal1, msg->pose.pose.position, "bb1");
  }
  void bb2_subscriber_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    this->box_bot_2_reached_goal =
        check_reached_goal(this->goal2, msg->pose.pose.position, "bb2");
  }
  void bb3_subscriber_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    this->box_bot_3_reached_goal =
        check_reached_goal(this->goal3, msg->pose.pose.position, "bb3");
  }

  void timer_callback() {
    std_msgs::msg::String message;
    RCLCPP_INFO(this->get_logger(), "BEGIN TIMER CALLBACK");
    if (this->box_bot_1_reached_goal) {
      RCLCPP_INFO(this->get_logger(), "BB1 reached goal!");
      message.data = "BB1 reached goal!";
      this->publisher1_->publish(message);
    };
    if (this->box_bot_2_reached_goal) {
      RCLCPP_INFO(this->get_logger(), "BB2 reached goal!");
      message.data = "BB2 reached goal!";
      this->publisher2_->publish(message);
    };
    if (this->box_bot_3_reached_goal) {
      RCLCPP_INFO(this->get_logger(), "BB3 reached goal!");
      message.data = "BB3 reached goal!";
      this->publisher3_->publish(message);
    };
    sleep(this->wait_time);

    RCLCPP_INFO(this->get_logger(), "END TIMER CALLBACK");
  }

  bool check_reached_goal(const Point goal, const Point current_pos,
                          const string bot, float error = 0.1) {
    float delta_x = fabs(goal.x - current_pos.x);
    float delta_y = fabs(goal.y - current_pos.y);
    if (delta_x <= error && delta_y <= error) {
      RCLCPP_INFO(this->get_logger(), "%s reached its goal!", bot.c_str());
      return true;
    }
    RCLCPP_INFO(this->get_logger(), "Error: dx=%f, dy=%f for %s", delta_x,
                delta_y, bot.c_str());
    return false;
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr bb1_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr bb2_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr bb3_subscription_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher1_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher2_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher3_;

  string bb1_topic_name;
  string bb2_topic_name;
  string bb3_topic_name;

  Point goal1;
  Point goal2;
  Point goal3;

  rclcpp::TimerBase::SharedPtr timer_;
  float wait_time;

  rclcpp::CallbackGroup::SharedPtr timer_callback_group;
  rclcpp::CallbackGroup::SharedPtr odom1_callback_group_;
  rclcpp::CallbackGroup::SharedPtr odom2_callback_group_;
  rclcpp::CallbackGroup::SharedPtr odom3_callback_group_;

  bool box_bot_1_reached_goal = false;
  bool box_bot_2_reached_goal = false;
  bool box_bot_3_reached_goal = false;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  string topic1 = "/box_bot_1/odom";
  string topic2 = "/box_bot_2/odom";
  string topic3 = "/box_bot_3/odom";

  Point goal1;
  goal1.x = 2.57;
  goal1.y = -1.092045;
  goal1.z = 0;

  Point goal2;
  goal2.x = 0.974281;
  goal2.y = -1.132045;
  goal2.z = 0;

  Point goal3;
  goal3.x = -0.507990;
  goal3.y = -1.132045;
  goal3.z = 0;

  float time = 1.0;

  std::shared_ptr<BoxBotManager> bb_man_node = std::make_shared<BoxBotManager>(
      topic1, topic2, topic3, goal1, goal2, goal3, time);
  RCLCPP_INFO(bb_man_node->get_logger(), "BoxBotManagerNode in Main...");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(bb_man_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}