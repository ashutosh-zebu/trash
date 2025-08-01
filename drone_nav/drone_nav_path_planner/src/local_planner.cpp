#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include "drone_nav_path_planner/srv/pos.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>

#include <mutex>

using namespace std::chrono_literals;

class localplanner : public rclcpp::Node     
{
public:
  localplanner() : Node("local_planner_server")
  {
    this->declare_parameter<double>("voxel_resolution", 0.2);
    this->declare_parameter<double>("path_footprint", 0.5);

    double voxel_resolution = this->get_parameter("voxel_resolution").as_double();
    path_footprint= this->get_parameter("path_footprint").as_double();
    
    RCLCPP_INFO(this->get_logger(),"%.2f",path_footprint);

    local_voxel_sub = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/local_occupancy_grid", 10, std::bind(&localplanner::local_voxel_callback, this, std::placeholders::_1));

    trajectory_sub = this->create_subscription<nav_msgs::msg::Path>(
        "/nav_trajectory", 10, std::bind(&localplanner::trajectory_callback, this, std::placeholders::_1));

    planner_client = this->create_client<drone_nav_path_planner::srv::Pos>("trajectory_planner");

    while (!planner_client->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for service...");
    }
  }

private:
  bool isCollision(float x, float y, float z) {

    resolution = temp_tree.getResolution();

    float step = resolution / 2.0;
    for (float dx = -path_footprint; dx <= path_footprint; dx += step) {
        for (float dy = -path_footprint; dy <= path_footprint; dy += step) {
            for (float dz = -path_footprint; dz <= path_footprint; dz += step) {
                if (std::sqrt(dx*dx + dy*dy + dz*dz) > path_footprint) continue;
                octomap::point3d p(x + dx, y + dy, z + dz);
                octomap::OcTreeNode* node1 = temp_tree.search(p);
                if (node1 == nullptr) {
                  continue;
                }
                bool occupied = node1 != nullptr && temp_tree.isNodeOccupied(node1);
              
                if (occupied) return true;
            }
        }
    }
    return false; 
  }

  void trajectory_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(tree_mutex_);

      for (const auto& pose_stamped : msg->poses) {
        double x = pose_stamped.pose.position.x;
        double y = pose_stamped.pose.position.y;
        double z = pose_stamped.pose.position.z;
        if (temp_tree.size() == 0) {
          return;
        }
        if (isCollision(x, y, z)){
          RCLCPP_WARN(this->get_logger(),"CAN COLLIDE");
          std::cout<<"can collide" << std::endl;

          auto request = std::make_shared<drone_nav_path_planner::srv::Pos::Request>();
          request->state = true;
          request->goal_position.x = 10.0;
          request->goal_position.y = 20.9;
          request->goal_position.z = 2.0;

          planner_client->async_send_request(request,
            [this](rclcpp::Client<drone_nav_path_planner::srv::Pos>::SharedFuture result) {
              RCLCPP_INFO(this->get_logger(), "Received response from planner");
            });
          break;
      }
      }
    }
  // void trajectory_callback(const nav_msgs::msg::Path::SharedPtr msg) {
  //   std::lock_guard<std::mutex> lock(tree_mutex_);

  //     for (const auto& pose_stamped : msg->poses) {
  //       double x = pose_stamped.pose.position.x;
  //       double y = pose_stamped.pose.position.y;
  //       double z = pose_stamped.pose.position.z;
  //       if (temp_tree.size() == 0) {
  //         return;
  //       }
  //       octomap::OcTreeNode* node1 = temp_tree.search(x, y, z);
  //       if (node1 == nullptr) {
  //         continue;
  //       }
  //       if (temp_tree.isNodeOccupied(node1)) {

  //         std::cout<<"can collide" << std::endl;

  //         auto request = std::make_shared<drone_nav_path_planner::srv::Pos::Request>();
  //         request->state = true;
  //         request->goal_position.x = 20.1;
  //         request->goal_position.y = 8.9;
  //         request->goal_position.z = 2.0;

  //         planner_client->async_send_request(request,
  //           [this](rclcpp::Client<drone_nav_path_planner::srv::Pos>::SharedFuture result) {
  //             RCLCPP_INFO(this->get_logger(), "Received response from planner");
  //           });
  //         break;
  //     }
  //     }
  //   }
  

  void local_voxel_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(tree_mutex_);
    temp_tree.clear();  

    for (const auto& marker : msg->markers) {
      for (const auto& pt : marker.points) {
        if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
          octomap::point3d p(pt.x, pt.y, pt.z);
          temp_tree.updateNode(p, true);  
        }
      }
    }
  }

  rclcpp::Client<drone_nav_path_planner::srv::Pos>::SharedPtr planner_client;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_sub;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr local_voxel_sub;
  float resolution;
  octomap::OcTree temp_tree{0.2};
  std::mutex tree_mutex_;
  double path_footprint;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<localplanner>());
  rclcpp::shutdown();
  return 0;
}
