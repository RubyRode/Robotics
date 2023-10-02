#include "rclcpp/rclcpp.hpp"
#include "fullnameservice/srv/name_srv.hpp"
#include <memory>
#include <cstdlib>
#include <string>

void concat(const std::shared_ptr<fullnameservice::srv::NameSrv::Request> request, std::shared_ptr<fullnameservice::srv::NameSrv::Response> response){
  response->full_name = std::string(request->name.c_str()) + " " + \
                        std::string(request->last_name.c_str()) + " " + \
                        std::string(request->first_name.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\n  name: %s\n  last_name: %s\n  first_name: %s",
              request->name.c_str(), request->last_name.c_str(), request->first_name.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back: %s", response->full_name.c_str());
        
}

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("service_name_node");
  rclcpp::Service<fullnameservice::srv::NameSrv>::SharedPtr service =
      node->create_service<fullnameservice::srv::NameSrv>("SummFullName", &concat);
  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to concat");
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  
  
}