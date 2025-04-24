#include <kinematic_path/kinematic_path.hpp>
#include <chrono>

namespace robot_kinematics
{

using namespace std::chrono_literals;

constexpr float WHEEL_RADIUS = 0.1;  // Wheel radius in meters
constexpr float CART_WIDTH = 2.0;    // Distance from the center of the left wheel to the center of the right wheel

KinematicPath::KinematicPath() : Node("diff_drive_velocity_publisher")
{
  wheel_vels_sub_ = this->create_subscription<kinematic_path::msg::WheelVelocities>(
      "/robot1/wheel_velocities", 10, std::bind(&KinematicPath::wheelVelocitiesCallback, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtlesim1/turtle1/cmd_vel", 10);
}

void KinematicPath::wheelVelocitiesCallback(const kinematic_path::msg::WheelVelocities::SharedPtr msg)
{
  left_wheel_angular_vel_ = msg->left_wheel_angular_vel;
  right_wheel_angular_vel_ = msg->right_wheel_angular_vel;
  publishVelocity();
}

void KinematicPath::publishVelocity()
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = WHEEL_RADIUS * (right_wheel_angular_vel_ + left_wheel_angular_vel_) / 2.0;
  cmd_vel.angular.z = WHEEL_RADIUS * (right_wheel_angular_vel_ - left_wheel_angular_vel_) / CART_WIDTH;
  cmd_vel_pub_->publish(cmd_vel);
}

}  // namespace robot_kinematics
