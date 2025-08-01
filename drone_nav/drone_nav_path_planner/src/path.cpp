#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include "drone_nav_path_planner/srv/pos.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <set>
#include <tuple>
#include <functional>
#include <cmath>
#include <stdexcept>
#include <map>
#include <iomanip>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <array>

std::string path = "/home/ashutosh/zebu_ws/src/drone_nav/drone_nav_path_planner/pcd/";
std::string octo_filename = path + "cloud_map.bt";
octomap::OcTree tree(octo_filename);

struct gn_structure{
  std::tuple<float,float,float> position;
  float g_cost;
};
std::vector<gn_structure> g_cost_and_position;

struct fn_structure{
  std::array<float,3> position;
  float f_cost;
};
std::vector<fn_structure> f_cost_and_position;

class globalplanner : public rclcpp::Node     
{
public:

  globalplanner() : Node("global_planner"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    // // test values 
    // std::array<float,3> s = {-0.1, -1.1, 5.3};
    // std::array<float,3> a = {20.1, 8.9, 8.9};

    // // for(auto i : s){
    // //   std::cout<<i<< std::endl;
    // // }
    // astar(s, a);

    this->declare_parameter<std::string>("planner_mode", "both");
    this->declare_parameter<double>("voxel_resolution", 0.2);
    this->declare_parameter<double>("path_footprint", 0.5);

    planner_mode = this->get_parameter("planner_mode").as_string();
    voxel_resolution = this->get_parameter("voxel_resolution").as_double();
    path_footprint = this->get_parameter("path_footprint").as_double();

    temp_tree.setResolution(voxel_resolution);
    // RCLCPP_INFO(this->get_logger(), "planner_mode: %s", planner_mode.c_str());
    // RCLCPP_INFO(this->get_logger(), "voxel_resolution: %.2f", voxel_resolution);
    // RCLCPP_INFO(this->get_logger(), "path_footprint: %.2f", path_footprint);


    path_publisher = this->create_publisher<nav_msgs::msg::Path>(
      "nav_trajectory",10
    );

    local_voxel_sub = this->create_subscription<visualization_msgs::msg::MarkerArray>("/local_occupancy_grid", 10, std::bind(&globalplanner::local_voxel_callback, this, std::placeholders::_1));
    
    planner_service = this->create_service<drone_nav_path_planner::srv::Pos>(
      "trajectory_planner",
      std::bind(&globalplanner::trajectory_planning, this, std::placeholders::_1, std::placeholders::_2)
    );
    timer_ = this->create_wall_timer(std::chrono::milliseconds(200),std::bind(&globalplanner::timerCallback, this));
  }

private:
  float heuristic(float x1, float y1, float z1, float x2, float y2, float z2){
    return (std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2) + std::pow(z2 - z1,2));
  }
  void smoothPath(std::vector<std::array<float, 3>>& path, int window_size = 3) {
    std::vector<std::array<float, 3>> smoothed;
    int n = path.size();

    for (int i = 0; i < n; ++i) {
        float sum_x = 0, sum_y = 0, sum_z = 0;
        int count = 0;
        for (int j = std::max(0, i - window_size); j <= std::min(n - 1, i + window_size); ++j) {
            sum_x += path[j][0];
            sum_y += path[j][1];
            sum_z += path[j][2];
            ++count;
        }
        smoothed.push_back({sum_x / count, sum_y / count, sum_z / count});
    }

    path = smoothed;
  }


  // bool isCollision(float x, float y, float z) {
  //   float path_footprint = 1.0; // meters
  //   float resolution = tree.getResolution();
  //   float step = resolution / 2.0;
  //   for (float dx = -path_footprint; dx <= path_footprint; dx += step) {
  //       for (float dy = -path_footprint; dy <= path_footprint; dy += step) {
  //           for (float dz = -path_footprint; dz <= path_footprint; dz += step) {
  //               if (std::sqrt(dx*dx + dy*dy + dz*dz) > path_footprint) continue;
  //               octomap::point3d p(x + dx, y + dy, z + dz);
  //               octomap::OcTreeNode* node = tree.search(p);
  //               if (node != nullptr && tree.isNodeOccupied(node)) {
  //                   return true;  
  //               }
  //           }
  //       }
  //   }
  //   return false; 
  // }

  bool isCollision(float x, float y, float z) {

    // float resolution = tree.getResolution();
    float step = voxel_resolution / 2.0;
    for (float dx = -path_footprint; dx <= path_footprint; dx += step) {
        for (float dy = -path_footprint; dy <= path_footprint; dy += step) {
            for (float dz = -path_footprint; dz <= path_footprint; dz += step) {
                if (std::sqrt(dx*dx + dy*dy + dz*dz) > path_footprint) continue;
                octomap::point3d p(x + dx, y + dy, z + dz);

                if(planner_mode == "local"){
                  octomap::OcTreeNode* node = temp_tree.search(p);
                  bool occupied = node != nullptr && temp_tree.isNodeOccupied(node);
                  if (occupied) return true;

                } if (planner_mode == "global"){
                  octomap::OcTreeNode* node = tree.search(p);
                  bool occupied = node != nullptr && tree.isNodeOccupied(node);
                  if (occupied) return true;

                } if (planner_mode == "both"){
                  octomap::OcTreeNode* node1 = tree.search(p);
                  octomap::OcTreeNode* node2 = temp_tree.search(p);
                  bool occupied1 = node1 != nullptr && tree.isNodeOccupied(node1);
                  bool occupied2 = node2 != nullptr && temp_tree.isNodeOccupied(node2);

                  if (occupied1 || occupied2) return true;
                }
                
            }
        }
    }
    return false; 
  }

  int astar(std::array<float,3>& initial_pos, std::array<float,3>& goal_pos) {
    path_nodes_.clear();
    RCLCPP_INFO(this->get_logger(), "Started Path Finding." );
    using prnode = std::pair<float, std::array<float, 3>>;
    auto cmp = [](prnode left, prnode right) { return left.first > right.first; };
    std::priority_queue<prnode, std::vector<prnode>, decltype(cmp)> unvisited_nodes(cmp);

    // float resolution = (float)tree.getResolution();
    float h_start = heuristic(initial_pos[0], initial_pos[1], initial_pos[2], goal_pos[0], goal_pos[1], goal_pos[2]);
    unvisited_nodes.push({h_start, initial_pos});

    std::map<std::tuple<float, float, float>, float> g_score;
    std::map<std::tuple<float, float, float>, float> f_score;
    std::map<std::tuple<float, float, float>, std::tuple<float, float, float>> came_from;
    std::set<std::tuple<float, float, float>> visited;

    std::tuple<float, float, float> start_key(initial_pos[0], initial_pos[1], initial_pos[2]);
    g_score[start_key] = 0.0;
    f_score[start_key] = h_start;

    int neighbors[26][3] = {
        {-1, 1, 1}, {0, 1, 1}, {1, 1, 1},
        {-1, 0, 1}, {0, 0, 1}, {1, 0, 1},
        {-1, -1, 1}, {0, -1, 1}, {1, -1, 1},
        {-1, 1, 0}, {0, 1, 0}, {1, 1, 0},
        {-1, 0, 0},             {1, 0, 0},
        {-1, -1, 0}, {0, -1, 0}, {1, -1, 0},
        {-1, 1, -1}, {0, 1, -1}, {1, 1, -1},
        {-1, 0, -1}, {0, 0, -1}, {1, 0, -1},
        {-1, -1, -1}, {0, -1, -1}, {1, -1, -1}
    };
    // int neighbors[6][3] = {
    //     {0,0,-1},{0,0,1},{0,-1,0},{-1,0,0},{0,1,0},{1,0,0}
    // };

    while (!unvisited_nodes.empty()) {
      if(planner_mode == "local" | planner_mode == "global" | planner_mode == "both"){
        auto [current_f, current] = unvisited_nodes.top();
        unvisited_nodes.pop();

        std::tuple<float, float, float> current_key(current[0], current[1], current[2]);
        float threshold = 0.5;

        if (heuristic(current[0], current[1], current[2], goal_pos[0], goal_pos[1], goal_pos[2]) < threshold) {
          RCLCPP_INFO(this->get_logger(), "Reached the goal." );
          while (came_from.find(current_key) != came_from.end()) {
              path_nodes_.push_back({std::get<0>(current_key), std::get<1>(current_key), std::get<2>(current_key)});
              current_key = came_from[current_key];
          }
          path_nodes_.push_back(initial_pos); 
          std::reverse(path_nodes_.begin(), path_nodes_.end());
          smoothPath(path_nodes_);
          return 1;
        }

        if (visited.count(current_key)) continue;
        visited.insert(current_key);

        for (int i = 0; i < 26; ++i) {
            float nx = std::round((current[0] + neighbors[i][0] * voxel_resolution) * 100.0) / 100.0;
            float ny = std::round((current[1] + neighbors[i][1] * voxel_resolution) * 100.0) / 100.0;
            float nz = std::round((current[2] + neighbors[i][2] * voxel_resolution) * 100.0) / 100.0;

            // std::cout<<nx<<"   ,   "<<ny<<"   ,   "<<nz<<std::endl;

            std::tuple<float, float, float> neighbor_key(nx, ny, nz);

            octomap::point3d p(nx, ny, nz);

            // octomap::OcTreeNode* node = tree.search(p);
            // if (node != nullptr && tree.isNodeOccupied(node)) continue;
            if (isCollision(nx, ny, nz)) continue;

            if (visited.count(neighbor_key)) continue;

            float tentative_g = g_score[current_key] + heuristic(current[0], current[1], current[2], nx, ny, nz);

            if (g_score.find(neighbor_key) == g_score.end() || tentative_g < g_score[neighbor_key]) {
                g_score[neighbor_key] = tentative_g;
                float h = heuristic(nx, ny, nz, goal_pos[0], goal_pos[1], goal_pos[2]);
                f_score[neighbor_key] = tentative_g + h;
                came_from[neighbor_key] = current_key;
                unvisited_nodes.push({f_score[neighbor_key], {nx, ny, nz}});
            }
        }
      
    } else{
      RCLCPP_ERROR(this->get_logger(),"INVALID PLANNER MODE.");
      return false; 
    }
    }
    RCLCPP_WARN(this->get_logger(),"Failed to find a path.");
    return -1;
  }

  void local_voxel_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(tree_mutex_);
    temp_tree.clear();  // clear previous data

    for (const auto& marker : msg->markers) {
        for (const auto& pt : marker.points) {
            if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
                octomap::point3d p(pt.x, pt.y, pt.z);
                temp_tree.updateNode(p, true);  // mark as occupied
            }
        }
    }
  }


  void timerCallback() {
    nav_msgs::msg::Path trajectory;
    trajectory.header.frame_id = "map";
    trajectory.header.stamp = this->now();
    trajectory.poses.clear();  
    for (const auto& pos : path_nodes_) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();
        pose.pose.position.x = pos[0];
        pose.pose.position.y = pos[1];
        pose.pose.position.z = pos[2];
        pose.pose.orientation.w = 1.0;
        
        trajectory.poses.push_back(pose);
    }

    path_publisher->publish(trajectory);
  }
  void trajectory_planning(
    const std::shared_ptr<drone_nav_path_planner::srv::Pos::Request> request,
    std::shared_ptr<drone_nav_path_planner::srv::Pos::Response> response)
    { 
      try{
      geometry_msgs::msg::TransformStamped transformStamped;
      if (tf_buffer_.canTransform("map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.5))) {
          transformStamped = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      } else {
          RCLCPP_WARN(this->get_logger(), "Transform not available yet");
          return;
      }
      // std::array<float,3> initial_pos = {request->initial_position.x, request->initial_position.y, request->initial_position.z};
      std::array<float,3> initial_pos = {transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z};
      std::array<float,3> goal_pos = {request->goal_position.x, request->goal_position.y, request->goal_position.z};
      // RCLCPP_INFO(this->get_logger(), "Received start: [%.2f, %.2f, %.2f], goal: [%.2f, %.2f, %.2f]",
      //             request->initial_position.x, request->initial_position.y, request->initial_position.z,
      //             request->goal_position.x, request->goal_position.y, request->goal_position.z);

      astar(initial_pos, goal_pos);
      response->success = true;
      }
      catch (tf2::TransformException &ex){
        RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
      }
    }
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
    std::vector<std::array<float, 3>> path_nodes_;
    rclcpp::Service<drone_nav_path_planner::srv::Pos>::SharedPtr planner_service;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr local_voxel_sub;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    octomap::OcTree temp_tree{0.5};
    std::mutex tree_mutex_;
    std::string planner_mode;
    double voxel_resolution;
    double path_footprint;

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);                       

  rclcpp::spin(std::make_shared<globalplanner>());  

  rclcpp::shutdown();                               

  return 0;
}