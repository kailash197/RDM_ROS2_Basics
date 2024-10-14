#include <rclcpp/rclcpp.hpp>
#include <unistd.h>

using namespace std::chrono_literals;

class TwoTimer : public rclcpp::Node {
public:
  TwoTimer(float sleep_timer1, float sleep_timer2)
      : Node("slow_timer_subscriber") {

    // Initialize one Reentrant callback group object
    callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    this->wait_time1 = sleep_timer1;
    this->wait_time2 = sleep_timer2;

    timer1_ = this->create_wall_timer(
        500ms, std::bind(&TwoTimer::timer1_callback, this), callback_group_);
    timer2_ = this->create_wall_timer(
        500ms, std::bind(&TwoTimer::timer2_callback, this), callback_group_);
  }

private:
  void timer1_callback() {
    RCLCPP_INFO(this->get_logger(), "Timer 1 Callback Start");
    sleep(this->wait_time1);
    RCLCPP_INFO(this->get_logger(), "Timer 1 Callback End");
  }

  void timer2_callback() {
    RCLCPP_INFO(this->get_logger(), "Timer 2 Callback Start");
    sleep(this->wait_time2);
    RCLCPP_INFO(this->get_logger(), "Timer 2 Callback End");
  }

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::TimerBase::SharedPtr timer2_;
  float wait_time1;
  float wait_time2;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Instantiate the Node
  float sleep_time1 = 1.0;
  float sleep_time2 = 3.0;
  std::shared_ptr<TwoTimer> two_timer_node =
      std::make_shared<TwoTimer>(sleep_time1, sleep_time2);

  // Initialize one MultiThreadedExecutor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(two_timer_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
/*
[INFO] [1649245219.694108351] [slow_timer_subscriber]: Timer 2 Callback Start
[INFO] [1649245219.698429480] [slow_timer_subscriber]: Timer 1 Callback End
[INFO] [1649245219.698525651] [slow_timer_subscriber]: Timer 1 Callback Start
[INFO] [1649245220.699197731] [slow_timer_subscriber]: Timer 1 Callback End
[INFO] [1649245220.699308479] [slow_timer_subscriber]: Timer 1 Callback Start
[INFO] [1649245221.699431638] [slow_timer_subscriber]: Timer 1 Callback End
[INFO] [1649245221.699605561] [slow_timer_subscriber]: Timer 1 Callback Start
[INFO] [1649245222.694242393] [slow_timer_subscriber]: Timer 2 Callback End
As you can see, callbacks are being executed without any restriction. This
means, even several instances of the same callback are being executed in
parallel. This behavior can be detected, for instance, because Timer 2 Callback,
which should be executed only once every 3 seconds, is being executed many more
times.

*/