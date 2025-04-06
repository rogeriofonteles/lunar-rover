#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Subscriber : public rclcpp::Node
{
public:
  Subscriber() : Node("rogerio_subscriber")
  {
    auto callback = [this](const std_msgs::msg::String::SharedPtr msg) {
      RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg->data << "'");
    };
    subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}