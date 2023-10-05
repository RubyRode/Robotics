#include <functional>
#include <memory>
#include <thread>
#include <cstring>

#include "action_messages/action/message_turtle_commands.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"


namespace action_turtle_commands{
class TurtleCmdActionServer : public rclcpp::Node{
public:
  
  using action_ms = action_messages::action::MessageTurtleCommands;
  using GoalHandleMessageTurtleCommands = rclcpp_action::ServerGoalHandle<action_ms>;

  void pose_callback(const turtlesim::msg::Pose& msg){
    cur_pose_ = msg;
  }
  
//  ACTION_TUTORIALS_CPP_PUBLIC
  explicit TurtleCmdActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
                                : Node("action_server", options){
    
    using namespace std::placeholders;
    this->action_server_ = rclcpp_action::create_server<action_ms>(
        this,
        "turtle_commands_execute",
        std::bind(&TurtleCmdActionServer::handle_goal, this, _1, _2),
        std::bind(&TurtleCmdActionServer::handle_cancel, this, _1),
        std::bind(&TurtleCmdActionServer::handle_accepted, this, _1));
    
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    subscription_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&TurtleCmdActionServer::pose_callback, this, _1));
    
  }

private:
  
  typedef struct {
    const std::shared_ptr<GoalHandleMessageTurtleCommands> goal_handle;
    geometry_msgs::msg::Twist request;
    const std::shared_ptr<action_ms::Feedback> feedback;
    const std::shared_ptr<action_ms::Result> result;
  }content;
  
  rclcpp_action::Server<action_ms>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
  
  turtlesim::msg::Pose init_pose_;
  turtlesim::msg::Pose cur_pose_;

//  Goal handler: gets a cmd and controls turtle through /turtle1/cmd_vel topic
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const action_ms::Goal> goal){
    
    RCLCPP_INFO(this->get_logger(), "Received goal request with command %s", goal->command.c_str());
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

  void turn(content& ctn, float theta){
    
    init_pose_ = cur_pose_;
    
    ctn.request.linear.x = 0;
    ctn.request.angular.z = theta/180*3.0;
    publisher_->publish(ctn.request);
    
    RCLCPP_INFO(this->get_logger(), "Turn %f", theta);
    ctn.result->result = true;
    
    
  }
  
  void fwd(content& ctn, float s, rclcpp::Rate& loop_rate){
    float odom = 0;
    init_pose_ = cur_pose_;
    auto request = geometry_msgs::msg::Twist();
    request.linear.x = s;
    publisher_->publish(request);
    
    while(abs(s - odom) > 0.02 && rclcpp::ok()){
      if (ctn.goal_handle->is_canceling()){
        ctn.result->result = false;
        publisher_->publish(geometry_msgs::msg::Twist());
        ctn.goal_handle->canceled(ctn.result);
        RCLCPP_INFO(this->get_logger(), "Goal cancelled, movement stopped");
        return;
      }
      
      odom = pow(pow(cur_pose_.x - init_pose_.x, 2) + pow(cur_pose_.y - init_pose_.y, 2), 0.5);
      ctn.feedback->odom = odom;
      ctn.goal_handle->publish_feedback(ctn.feedback);
      RCLCPP_INFO(this->get_logger(), "Publish Feedback %lf", odom);
      
      loop_rate.sleep();
    }
    RCLCPP_INFO(this->get_logger(), "Publish Feedback %lf", odom);
    
  }
  
  
  
  void execute(const std::shared_ptr<GoalHandleMessageTurtleCommands> goal_handle){
    
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(10);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<action_ms::Feedback>();
//    auto & sequence = feedback->odom;
    auto result = std::make_shared<action_ms::Result>();

    content ctn = {
      goal_handle,
      geometry_msgs::msg::Twist(),
      std::make_shared<action_ms::Feedback>(),
      std::make_shared<action_ms::Result>(),
      
    };
    
    if (goal->command == "turn_right"){
      turn(ctn, -goal->angle);
    }else if (goal->command == "turn_left"){
      turn(ctn, goal->angle);
    }else if (goal->command == "forward"){
      fwd(ctn, goal->s, loop_rate);
    }else if (strcmp(goal->command.c_str(), "") != 0){
      if (goal->s != 0){
        fwd(ctn, goal->s, loop_rate);
      }
      if (goal->angle != 0){
        turn(ctn, goal->angle);
      }
    }else{
      result->result = false;
      RCLCPP_INFO(this->get_logger(), "Unknown command or zero arguements %s", goal->command.c_str());
    }
    
    
    // Check if goal is done
    if (rclcpp::ok()) {
      result->result = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
       
    }else{
      result->result = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal failed");
    }
    
  }
  
  
};  // class FibonacciActionServer

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_turtle_commands::TurtleCmdActionServer);