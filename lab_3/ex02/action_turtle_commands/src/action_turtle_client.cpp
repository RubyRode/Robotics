#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "action_messages/action/message_turtle_commands.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_turtle_commands{
class TurtleCmdActionClient : public rclcpp::Node{
public:
  using action_ms = action_messages::action::MessageTurtleCommands;
  using GoalHandleMessageTurtleCommands = rclcpp_action::ClientGoalHandle<action_ms>;

  explicit TurtleCmdActionClient(std::shared_ptr<std::vector<action_messages::action::MessageTurtleCommands::Goal>> goals,
                                 const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("action_client", options){
    
    this->client_ptr_ = rclcpp_action::create_client<action_ms>(this, "turtle_commands_execute");

    if (!this->client_ptr_->wait_for_action_server()){
    
      RCLCPP_ERROR(this->get_logger(), "Action server is now not active");
      rclcpp::shutdown();
    
    }
  
    
    goals_ = goals;
    send_goal(goals->at(0));
  
  }

  void send_goal(action_ms::Goal g_msg)  {
    using namespace std::placeholders;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<action_ms>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&TurtleCmdActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&TurtleCmdActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&TurtleCmdActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(g_msg, send_goal_options);

  }

private:
  rclcpp_action::Client<action_ms>::SharedPtr client_ptr_;
  std::shared_ptr<std::vector<action_messages::action::MessageTurtleCommands::Goal>> goals_;
  size_t finished = 0;
  
  void goal_response_callback(const GoalHandleMessageTurtleCommands::SharedPtr & goal_handle){
    
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
    
  }

  void feedback_callback(GoalHandleMessageTurtleCommands::SharedPtr,
                         const std::shared_ptr<const action_ms::Feedback> feedback){
    
    RCLCPP_INFO(this->get_logger(), "Got distance %d", feedback->odom);
  
  }

  void result_callback(const GoalHandleMessageTurtleCommands::WrappedResult & result){
    
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Result: [%s]", result.result ? "true" : "false");
    finished++;
    
    if (finished != goals_->size()){
      send_goal(goals_->at(finished));
    }else{
      rclcpp::shutdown();
    }
    
    
  }
};  // class FibonacciActionClient

}  // namespace action_tutorials_cpp


int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  
  auto goals = std::make_shared<std::vector<action_messages::action::MessageTurtleCommands::Goal>>();

  auto goal_1 = action_messages::action::MessageTurtleCommands::Goal();
  goal_1.command = "forward";
  goal_1.s = 2;
  goal_1.angle = 0;
  goals->push_back(goal_1);

  auto goal_2 = action_messages::action::MessageTurtleCommands::Goal();
  goal_2.command = "turn_right";
  goal_2.s = 1;
  goal_2.angle = 90;
  goals->push_back(goal_2);

  auto client = std::make_shared<action_turtle_commands::TurtleCmdActionClient>(goals);

  rclcpp::spin(client);

  rclcpp::shutdown();
  return 0;    
  
  
}