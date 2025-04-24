#include <kinematic_path/kinematic_path.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_kinematics::KinematicPath>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}