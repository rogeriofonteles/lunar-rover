#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node
{
public:
  Publisher() : Node("rogerio_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(500ms, [this]() {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world!";
      RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.data << "'. Count = " << count_++);
      publisher_->publish(message);
    });
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint32_t count_;
};

int main()
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<Publisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
