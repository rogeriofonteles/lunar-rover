#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/publisher.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <kinematic_path/msg/wheel_velocities.hpp>

namespace robot_kinematics
{

class KinematicPath : public rclcpp::Node
{
public:
  KinematicPath();

private:
  void publishVelocity();
  void wheelVelocitiesCallback(const kinematic_path::msg::WheelVelocities::SharedPtr msg);

  float right_wheel_angular_vel_;
  float left_wheel_angular_vel_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<kinematic_path::msg::WheelVelocities>::SharedPtr wheel_vels_sub_;
};

}  // namespace robot_kinematics