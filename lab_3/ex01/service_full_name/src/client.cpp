#include "rclcpp/rclcpp.hpp"
#include "fullnameservice/srv/name_srv.hpp"
#include <memory>
#include <cstdlib>
#include <vector>
#include <string>
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  if (argc != 4){
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Not enough arguements: name, last_name, first_name");
    return 1;
  }
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("client_name_node");
  rclcpp::Client<fullnameservice::srv::NameSrv>::SharedPtr client =
    node->create_client<fullnameservice::srv::NameSrv>("SummFullName");
  
  std::vector<std::string> argvec;
  for (int i = 1; i < argc; i++){
    argvec.emplace_back(std::string(argv[i]));
  } 

  auto request = std::make_shared<fullnameservice::srv::NameSrv::Request>();
  request->name = argvec[0];
  request->last_name = argvec[1];
  request->first_name = argvec[2];
  
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
  
  auto result = client->async_send_request(request);
  
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS){
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", result.get()->full_name.c_str());
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service client_name_srv");
  }
  
  rclcpp::shutdown();
  return 0;
  
}