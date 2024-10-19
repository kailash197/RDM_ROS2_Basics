
#include "rclcpp/rclcpp.hpp"
#include "services_quiz_srv/srv/spin.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;

class ServiceServerRotateNode : public rclcpp::Node
{
public:
  ServiceServerRotateNode() : Node("rotate_server") {
    srv_ = create_service<services_quiz_srv::srv::Spin>("rotate", std::bind(&ServiceServerRotateNode::rotate_callback, this, _1, _2));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Service<services_quiz_srv::srv::Spin>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  void rotate_callback(
      const std::shared_ptr<services_quiz_srv::srv::Spin::Request> request,
      const std::shared_ptr<services_quiz_srv::srv::Spin::Response> response) {
        auto message = geometry_msgs::msg::Twist();  
        response->success = true;

        if (request->direction == "right") {   
            message.angular.z = -fabs(request->angular_velocity);        
        } else if (request->direction == "left") {
            message.angular.z = fabs(request->angular_velocity);
        } else {
            response->success = false;
            return;
        }
        
        // Publish the message to start rotating
        publisher_->publish(message);

        // Sleep for the specified time in seconds
        rclcpp::sleep_for(std::chrono::seconds(request->time));

        // Stop the rotation by publishing a zero velocity
        message.angular.z = 0.0;
        publisher_->publish(message);               
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServiceServerRotateNode>());
  rclcpp::shutdown();
  return 0;
}