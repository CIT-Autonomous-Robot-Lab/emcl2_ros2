#include "emcl2_node.h"

namespace emcl2 {

/*
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;


class MinimalPublisher : public rclcpp::Node
{
  public:
	MinimalPublisher();

  private:
	void timer_callback();
	
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	size_t count_;
};
*/

MinimalPublisher::MinimalPublisher() : Node("minimal_publisher"), count_(0)
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
    500ms, std::bind(&MinimalPublisher::timer_callback, this));
}

void MinimalPublisher::timer_callback()
{
	auto message = std_msgs::msg::String();
	message.data = "Hello, world! " + std::to_string(count_++);
	RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
	publisher_->publish(message);
}

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<emcl2::MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

