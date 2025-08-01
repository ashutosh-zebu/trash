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
  globalplanner() : Node("bidirectional_trajectory_publisher")
  {
    // test values 
    std::array<float,3> s = {0.1, -1.1, -0.1};
    std::array<float,3> a = {-89.3, -6.5, -1.1};

    // for(auto i : s){
    //   std::cout<<i<< std::endl;
    // }
    astar(s, a);
    path_publisher = this->create_publisher<nav_msgs::msg::Path>(
      "bi_nav_trajectory",10
    );
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
  int astar(std::array<float, 3>& initial_pos, std::array<float, 3>& goal_pos) {
    using prnode = std::pair<float, std::array<float, 3>>;
    auto cmp = [](prnode left, prnode right) { return left.first > right.first; };
    std::priority_queue<prnode, std::vector<prnode>, decltype(cmp)> start_open(cmp);
    std::priority_queue<prnode, std::vector<prnode>, decltype(cmp)> end_open(cmp);

    float resolution = static_cast<float>(tree.getResolution());

    float h_start = heuristic(initial_pos[0], initial_pos[1], initial_pos[2],
                              goal_pos[0], goal_pos[1], goal_pos[2]);
    float h_end = heuristic(goal_pos[0], goal_pos[1], goal_pos[2],
                            initial_pos[0], initial_pos[1], initial_pos[2]);

    start_open.push({h_start, initial_pos});
    end_open.push({h_end, goal_pos});

    using Key = std::tuple<float, float, float>;

    std::map<Key, float> start_g_score{{Key(initial_pos[0], initial_pos[1], initial_pos[2]), 0.0}};
    std::map<Key, float> end_g_score{{Key(goal_pos[0], goal_pos[1], goal_pos[2]), 0.0}};
    std::map<Key, Key> start_came_from, end_came_from;
    std::set<Key> start_visited, end_visited;

    const int neighbors[26][3] = {
        {-1, 1, 1}, {0, 1, 1}, {1, 1, 1}, {-1, 0, 1}, {0, 0, 1}, {1, 0, 1},
        {-1, -1, 1}, {0, -1, 1}, {1, -1, 1}, {-1, 1, 0}, {0, 1, 0}, {1, 1, 0},
        {-1, 0, 0}, {1, 0, 0}, {-1, -1, 0}, {0, -1, 0}, {1, -1, 0},
        {-1, 1, -1}, {0, 1, -1}, {1, 1, -1}, {-1, 0, -1}, {0, 0, -1}, {1, 0, -1},
        {-1, -1, -1}, {0, -1, -1}, {1, -1, -1}
    };

    auto reconstruct_bidirectional_path = [&](const Key& meeting_node) {
        std::vector<std::array<float, 3>> path;

        // Backtrack from meeting point to start
        Key node = meeting_node;
        while (start_came_from.count(node)) {
            path.push_back({std::get<0>(node), std::get<1>(node), std::get<2>(node)});
            node = start_came_from[node];
        }
        path.push_back(initial_pos);
        std::reverse(path.begin(), path.end());

        // Forward from meeting point to goal
        node = meeting_node;
        while (end_came_from.count(node)) {
            node = end_came_from[node];
            path.push_back({std::get<0>(node), std::get<1>(node), std::get<2>(node)});
        }

        path_nodes_ = path;
    };

    while (!start_open.empty() && !end_open.empty()) {
        if (!start_open.empty()) {
            auto [current_f, current] = start_open.top();
            start_open.pop();
            Key current_key(current[0], current[1], current[2]);

            if (end_visited.count(current_key)) {
                RCLCPP_INFO(this->get_logger(), "Meeting point found (from start side).");
                reconstruct_bidirectional_path(current_key);
                return 1;
            }

            if (start_visited.count(current_key)) continue;
            start_visited.insert(current_key);

            for (const auto& offset : neighbors) {
                float nx = std::round((current[0] + offset[0] * resolution) * 100.0f) / 100.0f;
                float ny = std::round((current[1] + offset[1] * resolution) * 100.0f) / 100.0f;
                float nz = std::round((current[2] + offset[2] * resolution) * 100.0f) / 100.0f;

                Key neighbor_key(nx, ny, nz);
                octomap::point3d p(nx, ny, nz);
                auto node = tree.search(p);
                if (node && tree.isNodeOccupied(node)) continue;
                if (start_visited.count(neighbor_key)) continue;

                float tentative_g = start_g_score[current_key] +
                    heuristic(current[0], current[1], current[2], nx, ny, nz);

                if (!start_g_score.count(neighbor_key) || tentative_g < start_g_score[neighbor_key]) {
                    start_g_score[neighbor_key] = tentative_g;
                    float h = heuristic(nx, ny, nz, goal_pos[0], goal_pos[1], goal_pos[2]);
                    start_came_from[neighbor_key] = current_key;
                    start_open.push({tentative_g + h, {nx, ny, nz}});
                }
            }
        }

        if (!end_open.empty()) {
            auto [current_f, current] = end_open.top();
            end_open.pop();
            Key current_key(current[0], current[1], current[2]);

            if (start_visited.count(current_key)) {
                RCLCPP_INFO(this->get_logger(), "Meeting point found (from goal side).");
                reconstruct_bidirectional_path(current_key);
                return 1;
            }

            if (end_visited.count(current_key)) continue;
            end_visited.insert(current_key);

            for (const auto& offset : neighbors) {
                float nx = std::round((current[0] + offset[0] * resolution) * 100.0f) / 100.0f;
                float ny = std::round((current[1] + offset[1] * resolution) * 100.0f) / 100.0f;
                float nz = std::round((current[2] + offset[2] * resolution) * 100.0f) / 100.0f;

                Key neighbor_key(nx, ny, nz);
                octomap::point3d p(nx, ny, nz);
                auto node = tree.search(p);
                if (node && tree.isNodeOccupied(node)) continue;
                if (end_visited.count(neighbor_key)) continue;

                float tentative_g = end_g_score[current_key] +
                    heuristic(current[0], current[1], current[2], nx, ny, nz);

                if (!end_g_score.count(neighbor_key) || tentative_g < end_g_score[neighbor_key]) {
                    end_g_score[neighbor_key] = tentative_g;
                    float h = heuristic(nx, ny, nz, initial_pos[0], initial_pos[1], initial_pos[2]);
                    end_came_from[neighbor_key] = current_key;
                    end_open.push({tentative_g + h, {nx, ny, nz}});
                }
            }
        }
    }

    RCLCPP_WARN(this->get_logger(), "Failed to find a path.");
    return -1;
}
//   int astar(std::array<float, 3>& initial_pos, std::array<float, 3>& goal_pos) {
//     using prnode = std::pair<float, std::array<float, 3>>;
//     auto cmp = [](prnode left, prnode right) { return left.first > right.first; };
//     std::priority_queue<prnode, std::vector<prnode>, decltype(cmp)> start_open(cmp);
//     std::priority_queue<prnode, std::vector<prnode>, decltype(cmp)> end_open(cmp);

//     float resolution = static_cast<float>(tree.getResolution());

//     float h_start = heuristic(initial_pos[0], initial_pos[1], initial_pos[2],
//                               goal_pos[0], goal_pos[1], goal_pos[2]);
//     float h_end = heuristic(goal_pos[0], goal_pos[1], goal_pos[2],
//                             initial_pos[0], initial_pos[1], initial_pos[2]);

//     start_open.push({h_start, initial_pos});
//     end_open.push({h_end, goal_pos});

//     using Key = std::tuple<float, float, float>;

//     std::map<Key, float> start_g_score{{Key(initial_pos[0], initial_pos[1], initial_pos[2]), 0.0}};
//     std::map<Key, float> end_g_score{{Key(goal_pos[0], goal_pos[1], goal_pos[2]), 0.0}};
//     std::map<Key, Key> start_came_from, end_came_from;
//     std::set<Key> start_visited, end_visited;

//     const int neighbors[26][3] = {
//         {-1, 1, 1}, {0, 1, 1}, {1, 1, 1}, {-1, 0, 1}, {0, 0, 1}, {1, 0, 1},
//         {-1, -1, 1}, {0, -1, 1}, {1, -1, 1}, {-1, 1, 0}, {0, 1, 0}, {1, 1, 0},
//         {-1, 0, 0}, {1, 0, 0}, {-1, -1, 0}, {0, -1, 0}, {1, -1, 0},
//         {-1, 1, -1}, {0, 1, -1}, {1, 1, -1}, {-1, 0, -1}, {0, 0, -1}, {1, 0, -1},
//         {-1, -1, -1}, {0, -1, -1}, {1, -1, -1}
//     };

//     auto reconstruct_bidirectional_path = [&](const Key& meeting_node) {
//         std::vector<std::array<float, 3>> path;

//         // Backtrack from meeting point to start
//         Key node = meeting_node;
//         while (start_came_from.count(node)) {
//             path.push_back({std::get<0>(node), std::get<1>(node), std::get<2>(node)});
//             node = start_came_from[node];
//         }
//         path.push_back(initial_pos);
//         std::reverse(path.begin(), path.end());

//         // Forward from meeting point to goal
//         node = meeting_node;
//         while (end_came_from.count(node)) {
//             node = end_came_from[node];
//             path.push_back({std::get<0>(node), std::get<1>(node), std::get<2>(node)});
//         }

//         path_nodes_ = path;
//     };

//     while (!start_open.empty() && !end_open.empty()) {
//         if (!start_open.empty()) {
//             auto [current_f, current] = start_open.top();
//             start_open.pop();
//             Key current_key(current[0], current[1], current[2]);

//             if (end_visited.count(current_key)) {
//                 RCLCPP_INFO(this->get_logger(), "Meeting point found (from start side).");
//                 reconstruct_bidirectional_path(current_key);
//                 return 1;
//             }

//             if (start_visited.count(current_key)) continue;
//             start_visited.insert(current_key);

//             for (const auto& offset : neighbors) {
//                 float nx = std::round((current[0] + offset[0] * resolution) * 100.0f) / 100.0f;
//                 float ny = std::round((current[1] + offset[1] * resolution) * 100.0f) / 100.0f;
//                 float nz = std::round((current[2] + offset[2] * resolution) * 100.0f) / 100.0f;

//                 Key neighbor_key(nx, ny, nz);
//                 octomap::point3d p(nx, ny, nz);
//                 auto node = tree.search(p);
//                 if (node && tree.isNodeOccupied(node)) continue;
//                 if (start_visited.count(neighbor_key)) continue;

//                 float tentative_g = start_g_score[current_key] +
//                     heuristic(current[0], current[1], current[2], nx, ny, nz);

//                 if (!start_g_score.count(neighbor_key) || tentative_g < start_g_score[neighbor_key]) {
//                     start_g_score[neighbor_key] = tentative_g;
//                     float h = heuristic(nx, ny, nz, goal_pos[0], goal_pos[1], goal_pos[2]);
//                     start_came_from[neighbor_key] = current_key;
//                     start_open.push({tentative_g + h, {nx, ny, nz}});
//                 }
//             }
//         }

//         if (!end_open.empty()) {
//             auto [current_f, current] = end_open.top();
//             end_open.pop();
//             Key current_key(current[0], current[1], current[2]);

//             if (start_visited.count(current_key)) {
//                 RCLCPP_INFO(this->get_logger(), "Meeting point found (from goal side).");
//                 reconstruct_bidirectional_path(current_key);
//                 return 1;
//             }

//             if (end_visited.count(current_key)) continue;
//             end_visited.insert(current_key);

//             for (const auto& offset : neighbors) {
//                 float nx = std::round((current[0] + offset[0] * resolution) * 100.0f) / 100.0f;
//                 float ny = std::round((current[1] + offset[1] * resolution) * 100.0f) / 100.0f;
//                 float nz = std::round((current[2] + offset[2] * resolution) * 100.0f) / 100.0f;

//                 Key neighbor_key(nx, ny, nz);
//                 octomap::point3d p(nx, ny, nz);
//                 auto node = tree.search(p);
//                 if (node && tree.isNodeOccupied(node)) continue;
//                 if (end_visited.count(neighbor_key)) continue;

//                 float tentative_g = end_g_score[current_key] +
//                     heuristic(current[0], current[1], current[2], nx, ny, nz);

//                 if (!end_g_score.count(neighbor_key) || tentative_g < end_g_score[neighbor_key]) {
//                     end_g_score[neighbor_key] = tentative_g;
//                     float h = heuristic(nx, ny, nz, initial_pos[0], initial_pos[1], initial_pos[2]);
//                     end_came_from[neighbor_key] = current_key;
//                     end_open.push({tentative_g + h, {nx, ny, nz}});
//                 }
//             }
//         }
//     }

//     RCLCPP_WARN(this->get_logger(), "Failed to find a path.");
//     return -1;
// }

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
      // std::array<float,3> initial_pos = {request->initial_position.x, request->initial_position.y, request->initial_position.z};
      std::array<float,3> goal_pos = {request->goal_position.x, request->goal_position.y, request->goal_position.z};
      // RCLCPP_INFO(this->get_logger(), "Received start: [%.2f, %.2f, %.2f], goal: [%.2f, %.2f, %.2f]",
      //             request->initial_position.x, request->initial_position.y, request->initial_position.z,
      //             request->goal_position.x, request->goal_position.y, request->goal_position.z);
      // astar(initial_pos, goal_pos);
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