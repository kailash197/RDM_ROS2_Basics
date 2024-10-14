#include <rclcpp/rclcpp.hpp>
#include <unistd.h>

using namespace std::chrono_literals;

class TwoTimer : public rclcpp::Node {
public:
  TwoTimer(float sleep_timer1, float sleep_timer2)
      : Node("slow_timer_subscriber") {

    this->wait_time1 = sleep_timer1;
    this->wait_time2 = sleep_timer2;

    timer1_ = this->create_wall_timer(
        500ms, std::bind(&TwoTimer::timer1_callback, this));
    timer2_ = this->create_wall_timer(
        500ms, std::bind(&TwoTimer::timer2_callback, this));
  }

private:
  void timer1_callback() {
    sleep(this->wait_time1);
    RCLCPP_INFO(this->get_logger(), "TIMER 1 CALLBACK");
  }

  void timer2_callback() {
    sleep(this->wait_time2);
    RCLCPP_INFO(this->get_logger(), "TIMER 2 CALLBACK");
  }

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

  // Initialize one executor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(two_timer_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

/*
[INFO] [1649243092.774541260] [slow_timer_subscriber]: TIMER 1 CALLBACK
[INFO] [1649243093.776366590] [slow_timer_subscriber]: TIMER 1 CALLBACK
[INFO] [1649243094.776625513] [slow_timer_subscriber]: TIMER 1 CALLBACK
[INFO] [1649243095.776895726] [slow_timer_subscriber]: TIMER 1 CALLBACK
[INFO] [1649243096.777163350] [slow_timer_subscriber]: TIMER 1 CALLBACK

For this particular code, the MultiThreadedExecutor is not creating multiple
threads for the node's callbacks because:

Both timer1_callback and timer2_callback are in the same node and default to
being treated as mutually exclusive. Even though the executor supports multiple
threads, the node's callback group prevents the callbacks from running in
parallel. Multi-Threaded Executor creates one thread per
Node if you do not specify otherwise.

The timer1_callback is executed because it has been instantiated first in the
constructor.

*/