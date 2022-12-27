#include "emcl2_node.h"

namespace emcl2 {

EMcl2Node::EMcl2Node() : Node("minimal_publisher"), count_(0)
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
    500ms, std::bind(&EMcl2Node::timer_callback, this));
}

void EMcl2Node::timer_callback()
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
  rclcpp::spin(std::make_shared<emcl2::EMcl2Node>());
  rclcpp::shutdown();
  return 0;
}

