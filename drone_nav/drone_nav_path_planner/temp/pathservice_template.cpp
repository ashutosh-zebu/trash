#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include "drone_nav_path_planner/srv/pos.hpp"

class GLOBALPLANNER : public rclcpp::Node     
{
public:
  GLOBALPLANNER() : Node("trajectory_publisher")
  {
    planner_service = this->create_service<drone_nav_path_planner::srv::Pos>(
      "trajectory_planner",
      std::bind(&GLOBALPLANNER::trajectory_planning, this, std::placeholders::_1, std::placeholders::_2)
    );
  }

private:
  void trajectory_planning(
    const std::shared_ptr<drone_nav_path_planner::srv::Pos::Request> request,
    std::shared_ptr<drone_nav_path_planner::srv::Pos::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received start: [%.2f, %.2f, %.2f], goal: [%.2f, %.2f, %.2f]",
                request->initial_position.x, request->initial_position.y, request->initial_position.z,
                request->goal_position.x, request->goal_position.y, request->goal_position.z);

    // Your path planning logic here...
    response->success = true;
  }

  rclcpp::Service<drone_nav_path_planner::srv::Pos>::SharedPtr planner_service;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);                       

  rclcpp::spin(std::make_shared<GLOBALPLANNER>());  

  rclcpp::shutdown();                               

  return 0;
}