#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "fairino_msgs/srv/remote_cmd_interface.hpp"
#include <sstream>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;
using FollowJT = control_msgs::action::FollowJointTrajectory;

class FollowJTServer : public rclcpp::Node
{
public:
  FollowJTServer()
  : Node("follow_joint_trajectory_server")
  {
    client_ = this->create_client<fairino_msgs::srv::RemoteCmdInterface>("/fairino_remote_command_service");

    action_server_ = rclcpp_action::create_server<FollowJT>(
      this,
      "moveit_joint_controller/follow_joint_trajectory",
      std::bind(&FollowJTServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&FollowJTServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&FollowJTServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "FollowJointTrajectory action server started.");
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJT::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received trajectory with %zu points.", goal->trajectory.points.size());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJT>>)
  {
    RCLCPP_WARN(this->get_logger(), "Cancel requested.");
    send_command("StopMotion()");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJT>> goal_handle)
  {
    std::thread([this, goal_handle]() {
      execute(goal_handle);
    }).detach();
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJT>> goal_handle)
  {
    auto goal = goal_handle->get_goal();
    const auto & traj = goal->trajectory.points;
    if (traj.empty()) return;

    send_command("RobotEnable(1)");
    send_command("Mode(0)");
    send_command("SetSpeed(20)");

    int id = 1;
    for (auto & point : traj)
    {
      if (goal_handle->is_canceling()) {
        send_command("StopMotion()");
        goal_handle->canceled(std::make_shared<FollowJT::Result>());
        return;
      }

      // Converti radianti → gradi
      std::ostringstream cmd;
      cmd << "JNTPoint(" << id << ",";
      for (size_t j = 0; j < point.positions.size(); ++j)
      {
        cmd << point.positions[j] * 180.0 / M_PI;
        if (j < point.positions.size()-1) cmd << ",";
      }
      cmd << ")";
      send_command(cmd.str());

      std::ostringstream move;
      move << "MoveJ(JNT" << id << ",20,0,0)";
      send_command(move.str());

      id++;
      std::this_thread::sleep_for(50ms);
    }

    auto result = std::make_shared<FollowJT::Result>();
    result->error_code = FollowJT::Result::SUCCESSFUL;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Trajectory execution complete.");
  }

  void send_command(const std::string & cmd)
  {
  auto req = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
  req->cmd_str = cmd;

  if (!client_->wait_for_service(1s))
  {
      RCLCPP_ERROR(this->get_logger(), "Service not available: %s", cmd.c_str());
      return;   
  }

  auto future = client_->async_send_request(req,
  [this, cmd](rclcpp::Client<fairino_msgs::srv::RemoteCmdInterface>::SharedFuture result) {
    try {
          (void)result.get();  // opzionale, se non serve il valore
          RCLCPP_DEBUG(this->get_logger(), "Command executed successfully: %s", cmd.c_str());
        } catch (...) {
          RCLCPP_ERROR(this->get_logger(), "Command failed: %s", cmd.c_str());
        }
        });
  }




  rclcpp::Client<fairino_msgs::srv::RemoteCmdInterface>::SharedPtr client_;
  rclcpp_action::Server<FollowJT>::SharedPtr action_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<FollowJTServer>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
