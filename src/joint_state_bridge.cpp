#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "fairino_msgs/msg/robot_nonrt_state.hpp"
#include <cmath>

class JointStateBridge : public rclcpp::Node
{
public:
  JointStateBridge() : Node("joint_state_bridge")
  {
    state_sub_ = this->create_subscription<fairino_msgs::msg::RobotNonrtState>(
      "/nonrt_state_data", 10,
      std::bind(&JointStateBridge::state_callback, this, std::placeholders::_1));

    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    joint_names_ = {"joint1","joint2","joint3","joint4","joint5","joint6"};
  }

private:
  void state_callback(const fairino_msgs::msg::RobotNonrtState::SharedPtr msg)
  {
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.name = joint_names_;

    // conversione in radianti
    js.position = {
      msg->j1_cur_pos * M_PI / 180.0,
      msg->j2_cur_pos * M_PI / 180.0,
      msg->j3_cur_pos * M_PI / 180.0,
      msg->j4_cur_pos * M_PI / 180.0,
      msg->j5_cur_pos * M_PI / 180.0,
      msg->j6_cur_pos * M_PI / 180.0
    };

    joint_pub_->publish(js);
  }

  rclcpp::Subscription<fairino_msgs::msg::RobotNonrtState>::SharedPtr state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  std::vector<std::string> joint_names_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateBridge>());
  rclcpp::shutdown();
  return 0;
}
