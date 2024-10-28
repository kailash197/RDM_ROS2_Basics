#include <inttypes.h>
#include <memory>
#include <string>
#include <iostream>

#include "actions_quiz_msg/action/distance.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using Distance = actions_quiz_msg::action::Distance;
using GoalHandle = rclcpp_action::ClientGoalHandle<Distance>;
using namespace std::placeholders;

class DistanceActionClient : public rclcpp::Node
{
public:
  explicit DistanceActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("distance_action_client", node_options), goal_done_(false) {
    this->client_ptr_ = rclcpp_action::create_client<Distance>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      this->action_name);
  }
  bool is_goal_done() const { return this->goal_done_; }
  void send_goal();
  
private:
  rclcpp_action::Client<Distance>::SharedPtr client_ptr_;
  std::string action_name = "distance_as";
  bool goal_done_;

  void goal_response_callback(const GoalHandle::SharedPtr & goal_handle);
  void feedback_callback(GoalHandle::SharedPtr,
    const std::shared_ptr<const Distance::Feedback> feedback);
  void result_callback(const GoalHandle::WrappedResult & result);

};

void DistanceActionClient::send_goal() {
    using namespace std::placeholders;
    this->goal_done_ = false;
    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = Distance::Goal();
    goal_msg.seconds = 20;
    RCLCPP_INFO(this->get_logger(), "Sending goal: %d seconds.", goal_msg.seconds);

    auto send_goal_options = rclcpp_action::Client<Distance>::SendGoalOptions();
                
    send_goal_options.goal_response_callback =
      std::bind(&DistanceActionClient::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
      std::bind(&DistanceActionClient::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
      std::bind(&DistanceActionClient::result_callback, this, _1);
      
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

void DistanceActionClient::goal_response_callback(const GoalHandle::SharedPtr &goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void DistanceActionClient::feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const Distance::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "current_dist: %f", feedback->current_dist);
}

void DistanceActionClient::result_callback(const GoalHandle::WrappedResult &result) {
    this->goal_done_ = true;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_ERROR(this->get_logger(), "Goal finished with status: SUCCEEDED");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Result received: status: %s", result.result->status? "True":"False");
    RCLCPP_INFO(this->get_logger(), "Result received: total_dist: %f", result.result->total_dist);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<DistanceActionClient>();    
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_client);
  action_client->send_goal();

  while (!action_client->is_goal_done()) {
    executor.spin_some();
  }

  rclcpp::shutdown();
  return 0;
}