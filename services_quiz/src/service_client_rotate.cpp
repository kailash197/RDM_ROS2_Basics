#include "rclcpp/rclcpp.hpp"
#include "services_quiz_srv/srv/spin.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <iostream>

using namespace std::chrono_literals;
using Spin = services_quiz_srv::srv::Spin;

class ServiceClientRotateNode : public rclcpp::Node {
  private:
  rclcpp::Client<Spin>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool service_done_ = false;
  bool service_called_ = false;

  void timer_callback() {

    if (!service_called_) {
      RCLCPP_INFO(this->get_logger(), "Send Async Request");
      send_async_request();
    } else {
      RCLCPP_INFO(this->get_logger(), "Timer Callback Executed");
    }
  }

  void send_async_request() {
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Client interrupted while waiting for service. Terminating...");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "/rotate Service Unavailable. Waiting for Service...");
    }
    auto request = std::make_shared<Spin::Request>();
    request->direction = "right";
    request->angular_velocity = 0.20;
    request->time = 10;

    auto result_future = client_->async_send_request(
        request, std::bind(&ServiceClientRotateNode::response_callback, this,
                           std::placeholders::_1));
    service_called_ = true;

    // Now check for the response after a timeout of 1 second
    auto status = result_future.wait_for(1s);
    if (status != std::future_status::ready) {
      RCLCPP_WARN(this->get_logger(), "Response not ready yet.");
    }
  }

  void response_callback(rclcpp::Client<Spin>::SharedFuture future) {
    // Get response value
    auto result = future.get();
    if (result->success == true) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned success");
    } else if (result->success == false) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned false");
    }
    service_done_ = true;
  }

public:
  ServiceClientRotateNode() : Node("rotate_client") {
    client_ = this->create_client<Spin>("rotate");
    timer_ = this->create_wall_timer(1s, std::bind(&ServiceClientRotateNode::timer_callback, this));
  }
  bool is_service_done() const { return this->service_done_; }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto service_client = std::make_shared<ServiceClientRotateNode>();
  while (!service_client->is_service_done()) {
    rclcpp::spin_some(service_client);
  }
  rclcpp::shutdown();
  return 0;
}