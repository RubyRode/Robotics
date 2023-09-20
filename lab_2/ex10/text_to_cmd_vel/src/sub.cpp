#include <memory>
#include <string.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
using std::placeholders::_1;


class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("t_cmd_vel_sub")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "cmd_text", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    }

  private:

    

    void topic_callback(const std_msgs::msg::String & msg) const
    {
      std::string string_msg = msg.data.c_str();
      std::string T_R = "turn_right";
      std::string T_L = "turn_left";
      std::string M_F = "move_forward";
      std::string M_B = "move_backward";
      if (T_R.compare(string_msg) == 0/*strcmp(string_msg, T_R)*/){
        geometry_msgs::msg::Twist twt_msg;
        twt_msg.linear.x = 0.0;
        twt_msg.linear.y = 0.0;
        twt_msg.linear.z = 0.0;
        twt_msg.angular.x = 0.0;
        twt_msg.angular.y = 0.0;
        twt_msg.angular.z = -1.55;
        publisher_->publish(twt_msg);
        RCLCPP_DEBUG(this->get_logger(), "%s (linear: %f, %f, %f | angular: %f, %f, %f)", msg.data.c_str(),
          twt_msg.linear.x, twt_msg.linear.y, twt_msg.linear.z,
          twt_msg.angular.x, twt_msg.angular.y, twt_msg.angular.z);
        RCLCPP_INFO(this->get_logger(), "%s", msg.data.c_str());
      }
      else if (T_L.compare(string_msg) == 0/*strcmp(string_msg, T_L)*/){
        geometry_msgs::msg::Twist twt_msg;
        twt_msg.linear.x = 0.0;
        twt_msg.linear.y = 0.0;
        twt_msg.linear.z = 0.0;
        twt_msg.angular.x = 0.0;
        twt_msg.angular.y = 0.0;
        twt_msg.angular.z = 1.55;
        publisher_->publish(twt_msg);
        RCLCPP_DEBUG(this->get_logger(), "%s (linear: %f, %f, %f | angular: %f, %f, %f)", msg.data.c_str(),
          twt_msg.linear.x, twt_msg.linear.y, twt_msg.linear.z,
          twt_msg.angular.x, twt_msg.angular.y, twt_msg.angular.z);
        RCLCPP_INFO(this->get_logger(), "%s", msg.data.c_str());
      }
      else if (M_F.compare(string_msg) == 0/*strcmp(string_msg, M_F)*/){
        geometry_msgs::msg::Twist twt_msg;
        twt_msg.linear.x = 1.0;
        twt_msg.linear.y = 0.0;
        twt_msg.linear.z = 0.0;
        twt_msg.angular.x = 0.0;
        twt_msg.angular.y = 0.0;
        twt_msg.angular.z = 0.0;
        publisher_->publish(twt_msg);
        RCLCPP_DEBUG(this->get_logger(), "%s (linear: %f, %f, %f | angular: %f, %f, %f)", msg.data.c_str(),
          twt_msg.linear.x, twt_msg.linear.y, twt_msg.linear.z,
          twt_msg.angular.x, twt_msg.angular.y, twt_msg.angular.z);
        RCLCPP_INFO(this->get_logger(), "%s", msg.data.c_str());
      }
      else if (M_B.compare(string_msg) == 0/*strcmp(string_msg, M_B)*/){
        geometry_msgs::msg::Twist twt_msg;
        twt_msg.linear.x = -1.0;
        twt_msg.linear.y = 0.0;
        twt_msg.linear.z = 0.0;
        twt_msg.angular.x = 0.0;
        twt_msg.angular.y = 0.0;
        twt_msg.angular.z = 0.0;
        publisher_->publish(twt_msg);
        RCLCPP_DEBUG(this->get_logger(), "%s (linear: %.2f, %.2f, %.2f | angular: %.2f, %.2f, %.2f)", msg.data.c_str(),
          twt_msg.linear.x, twt_msg.linear.y, twt_msg.linear.z,
          twt_msg.angular.x, twt_msg.angular.y, twt_msg.angular.z);
        RCLCPP_INFO(this->get_logger(), "%s", msg.data.c_str());
      }
      else{
          RCLCPP_INFO(this->get_logger(), "Command undefined '%s'", msg.data.c_str());
      }
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}