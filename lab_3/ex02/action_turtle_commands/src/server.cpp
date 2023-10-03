#include <functional>
#include <memory>
#include <thread>

#include "action_messages/action/message_turtle_commands.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"


namespace action_turtle_commands{
class TurtleCmdActionServer : public rclcpp::Node{
public:
  
  using action_ms = action_messages::action::MessageTurtleCommands;
  using GoalHandleMessageTurtleCommands = rclcpp_action::ServerGoalHandle<action_ms>;

//  ACTION_TUTORIALS_CPP_PUBLIC
  explicit TurtleCmdActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
                                : Node("action_server_node", options){
    
    using namespace std::placeholders;
    this->action_server_ = rclcpp_action::create_server<action_ms>(
        this,
        "action_server",
        std::bind(&TurtleCmdActionServer::handle_goal, this, _1, _2),
        std::bind(&TurtleCmdActionServer::handle_cancel, this, _1),
        std::bind(&TurtleCmdActionServer::handle_accepted, this, _1));
    
  }

private:
  
  rclcpp_action::Server<action_ms>::SharedPtr action_server_;

//  Goal handler: gets a cmd and controls turtle through /turtle1/cmd_vel topic
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const action_ms::Goal> goal){
    
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %s", goal->command.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleMessageTurtleCommands> goal_handle){
    
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMessageTurtleCommands> goal_handle){
    
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&TurtleCmdActionServer::execute, this, _1), goal_handle}.detach();
  
  }

  void execute(const std::shared_ptr<GoalHandleMessageTurtleCommands> goal_handle){
    
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<action_ms::Feedback>();
    auto & sequence = feedback->odom;
    auto result = std::make_shared<action_ms::Result>();

    
    
    
//    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
//      // Check if there is a cancel request
//      if (goal_handle->is_canceling()) {
//        result->sequence = sequence;
//        goal_handle->canceled(result);
//        RCLCPP_INFO(this->get_logger(), "Goal canceled");
//        return;
//      }
//      // Update sequence
//      sequence.push_back(sequence[i] + sequence[i - 1]);
//      // Publish feedback
//      goal_handle->publish_feedback(feedback);
//      RCLCPP_INFO(this->get_logger(), "Publish feedback");
//
//      loop_rate.sleep();
//    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->result = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
    
  }
  
  
};  // class FibonacciActionServer

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_turtle_commands::TurtleCmdActionServer);