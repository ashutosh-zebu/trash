#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include "drone_nav_path_planner/srv/pos.hpp"

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
  globalplanner() : Node("trajectory_publisher")
  {
    // test values 
    std::array<float,3> s = {0.1, -1.1, -0.1};
    std::array<float,3> a = {-89.3, -6.5, -1.1};

    // for(auto i : s){
    //   std::cout<<i<< std::endl;
    // }
    astar(s, a);
    path_publisher = this->create_publisher<nav_msgs::msg::Path>(
      "nav_trajectory",10
    );
    planner_service = this->create_service<drone_nav_path_planner::srv::Pos>(
      "trajectory_planner",
      std::bind(&globalplanner::trajectory_planning, this, std::placeholders::_1, std::placeholders::_2)
    );
    timer_ = this->create_wall_timer(std::chrono::milliseconds(200),std::bind(&globalplanner::timerCallback, this));
  }

private:
  float heuristic(float x1, float y1, float z1, float x2, float y2, float z2){
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2) + std::pow(z2 - z1,2));
  }

  int astar(std::array<float,3>& initial_pos, std::array<float,3>& goal_pos) {
    using prnode = std::pair<float, std::array<float, 3>>;
    auto cmp = [](prnode left, prnode right) { return left.first > right.first; };
    std::priority_queue<prnode, std::vector<prnode>, decltype(cmp)> unvisited_nodes(cmp);

    float resolution = (float)tree.getResolution();
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

    while (!unvisited_nodes.empty()) {
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

          return 1;
        }

        if (visited.count(current_key)) continue;
        visited.insert(current_key);

        for (int i = 0; i < 26; ++i) {
            float nx = std::round((current[0] + neighbors[i][0] * resolution) * 100.0) / 100.0;
            float ny = std::round((current[1] + neighbors[i][1] * resolution) * 100.0) / 100.0;
            float nz = std::round((current[2] + neighbors[i][2] * resolution) * 100.0) / 100.0;

            std::cout<<nx<<"   ,   "<<ny<<"   ,   "<<nz<<std::endl;

            std::tuple<float, float, float> neighbor_key(nx, ny, nz);

            octomap::point3d p(nx, ny, nz);
            octomap::OcTreeNode* node = tree.search(p);

            if (node != nullptr && tree.isNodeOccupied(node)) continue;
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
    }
    RCLCPP_WARN(this->get_logger(),"Failed to find a path.");
    return -1;
  }

  void timerCallback() {
    nav_msgs::msg::Path trajectory;
    trajectory.header.frame_id = "map";
    trajectory.header.stamp = this->now();

    for (const auto& pos : path_nodes_) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();
        pose.pose.position.x = pos[0];
        pose.pose.position.y = pos[1];
        pose.pose.position.z = pos[2];
        pose.pose.orientation.w = 1.0;  // default orientation
        trajectory.poses.push_back(pose);
    }

    path_publisher->publish(trajectory);
  }
  void trajectory_planning(
    const std::shared_ptr<drone_nav_path_planner::srv::Pos::Request> request,
    std::shared_ptr<drone_nav_path_planner::srv::Pos::Response> response)
    {
      std::array<float,3> initial_pos = {request->initial_position.x, request->initial_position.y, request->initial_position.z};
      std::array<float,3> goal_pos = {request->goal_position.x, request->goal_position.y, request->goal_position.z};
      // RCLCPP_INFO(this->get_logger(), "Received start: [%.2f, %.2f, %.2f], goal: [%.2f, %.2f, %.2f]",
      //             request->initial_position.x, request->initial_position.y, request->initial_position.z,
      //             request->goal_position.x, request->goal_position.y, request->goal_position.z);
      astar(initial_pos, goal_pos);
      response->success = true;
    }
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
    std::vector<std::array<float, 3>> path_nodes_;
    rclcpp::Service<drone_nav_path_planner::srv::Pos>::SharedPtr planner_service;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);                       

  rclcpp::spin(std::make_shared<globalplanner>());  

  rclcpp::shutdown();                               

  return 0;
}