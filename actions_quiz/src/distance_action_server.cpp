#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "actions_quiz_msg/action/distance.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"

using Distance = actions_quiz_msg::action::Distance;
using GoalHandle = rclcpp_action::ServerGoalHandle<Distance>;
using namespace std::placeholders;

class DistanceActionServer : public rclcpp::Node {
  public:
    explicit DistanceActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()):
      Node("distance_action_server", options) {
        this->action_server_ = rclcpp_action::create_server<Distance>(
          this, "distance_as", 
          std::bind(&DistanceActionServer::handle_goal, this, _1, _2),
          std::bind(&DistanceActionServer::handle_cancel, this, _1),
          std::bind(&DistanceActionServer::handle_accepted, this, _1));
        this->subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "odom", 10,std::bind(&DistanceActionServer::odom_subscriber_callback, this, _1));
    cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    topic_publisher_ = this->create_publisher<std_msgs::msg::Float64>("total_distance", 10);
    this->move_.linear.x = 0.2;
    this->move_.angular.z = 0.2;
    RCLCPP_INFO(this->get_logger(), "Action Server Initialized.");
  }


private:
  rclcpp_action::Server<Distance>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr topic_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  
  geometry_msgs::msg::Twist move_;
  nav_msgs::msg::Odometry prev_pose_;
  double distance_;


  void odom_subscriber_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double dx = msg->pose.pose.position.x - prev_pose_.pose.pose.position.x;
    double dy = msg->pose.pose.position.y - prev_pose_.pose.pose.position.y;
    double dz = msg->pose.pose.position.z - prev_pose_.pose.pose.position.z;
    double distance = sqrt(dx * dx + dy * dy + dz * dz);
    this->distance_ += distance;
    this->prev_pose_.pose = msg->pose;
    cmd_publisher_->publish(move_);
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Distance::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Action Requested: Goal Seconds %d", goal->seconds);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Action Canceled.");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&DistanceActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    this->distance_ = 0.0;
    auto feedback = std::make_shared<Distance::Feedback>();
    auto result = std::make_shared<Distance::Result>();
    rclcpp::Rate loop_rate(1);
    std_msgs::msg::Float64 message;

    for (int i = 0; (i <= goal->seconds) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->status = false;
        result->total_dist = -1.0;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Action Canceled while in execution.");
        return;
      }

      feedback->current_dist = this->distance_;
      goal_handle->publish_feedback(feedback);
      message.data = this->distance_;
      topic_publisher_->publish(message);
      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->status = true;
      result->total_dist = this->distance_;      
      goal_handle->succeed(result);
      message.data = this->distance_;
      topic_publisher_->publish(message);
      RCLCPP_INFO(this->get_logger(), "Action Completed.");
    }
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<DistanceActionServer>();    
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
