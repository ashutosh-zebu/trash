#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
from nav_msgs.msg import Path
import rclpy.time
import os
import yaml
import octomap

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped

import numpy as np
import heapq
import math
import time

from scipy.interpolate import make_interp_spline


# import Astar.Astar_copy as A

class path_planner(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')

        pkg_share = get_package_share_directory('path_planner')

        path = os.path.join(pkg_share, 'config', 'position.yaml')
        with open(path,"r") as f:
            self.position = yaml.safe_load(f)

        # https://octomap.github.io/octomap/doc/classoctomap_1_1OcTreeNode.html
        tree_path = os.path.join(pkg_share, 'pcd', 'cloud_map.bt')
        self.tree = octomap.OcTree(tree_path.encode('utf-8'))
        tree_resolution = self.tree.getResolution()
        bbmin, bbmax = self.tree.getMetricMin(), self.tree.getMetricMax()
        # print(bbmin,bbmax)
        print(self.position['initial_position'], self.position['goal_position'])
        grid_size = self.tree.getMetricSize()
        free_node = []
        occupied_node = []

        # Iterate over all unoccupied voxels
        for node in self.tree.begin_leafs():
            coord = tuple(self.tree.keyToCoord(node.getKey()))
            if not self.tree.isNodeOccupied(node):
                free_node.append(coord)
            else:
                occupied_node.append(coord)

        self.trajectory_points = self.astar(grid = free_node, 
                                         start= tuple(self.position['initial_position']), 
                                         goal= tuple(self.position['goal_position']), 
                                         resolution=tree_resolution,
                                         size = grid_size
                                         )
        
        # points = np.array(self.trajectory_points)  

        # t = np.linspace(0, 1, len(points))

        # num_interp_points = 300
        # t_smooth = np.linspace(0, 1, num_interp_points)

        # x_spline = make_interp_spline(t, points[:, 0], k=3)(t_smooth)
        # y_spline = make_interp_spline(t, points[:, 1], k=3)(t_smooth)
        # z_spline = make_interp_spline(t, points[:, 2], k=3)(t_smooth)

        # smoothed_path = np.vstack((x_spline, y_spline, z_spline)).T  

        # self.smoothed_trajectory = smoothed_path.tolist()
        
        
        
        if self.trajectory_points is None:
            self.get_logger().info("no path found ...")
            exit(0)
        else:
            print(self.trajectory_points)
            self.poses = self.list_to_pose_stamped(self.trajectory_points)
    #     # if self.trajectory_points is None:
    #     #     print("None")
    #     # else:
    #     #     for i in self.trajectory_points:
    #     #         print(i)

        self.pub = self.create_publisher(Path, "/nav_trajectory_py", 10)
        process_rate = 0.1
        self.timer = self.create_timer(process_rate, self.planner)

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])# we are using manhattan distance

    def heuristic_eucledian(self, a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2 + (a[2] - b[2])**2)

    def astar(self, grid, start, goal, resolution, size):
        # print(f"{len(grid)}  ,  {start}  , {goal}  ,  {resolution}  ,  {size}")
        open_set = []                               # unvisited nodes  (min heap)
        closed_set = set()                          # visited nodes currently an empty set because no node visited
        came_from ={}
        heapq.heappush(open_set, (0, start))        # inserts a tuple((0, start)) into the heap(open_set)
        
        g_n = {start : 0}
        f_n = {start:g_n[start] + self.heuristic_eucledian(start,goal)}            # for the starting f(n) = g(n) + h(n)
        neighbor_cell = [(-1,1,1),(0,1,1),(1,1,1),(-1,0,1),(0,0,1),(1,0,1),(-1,-1,1),(0,-1,1),(1,-1,1),
                         (-1,1,0),(0,1,0),(1,1,0),(-1,0,0),(1,0,0),(-1,-1,0),(0,-1,0),(1,-1,0),
                         (-1,1,-1),(0,1,-1),(1,1,-1),(-1,0,-1),(0,0,-1),(1,0,-1),(-1,-1,-1),(0,-1,-1),(1,-1,-1)
                         ]

        # neighbor_cell = [(0,0,1),(0,1,0),(-1,0,0),(1,0,0),(0,-1,0),(0,0,-1),
        #                 ]
        
        # neighbor_cell = [(0,1,1),(-1,0,1),(0,0,1),(1,0,1),(0,-1,1),(-1,1,0),(0,1,0),(1,1,0),(-1,0,0),(1,0,0),(-1,-1,0),(0,-1,0),(1,-1,0),(0,1,-1),(-1,0,-1),(0,0,-1),(1,0,-1),(0,-1,-1),
        #                  ]
        
        initial = time.time()
        
        while open_set:     
           
            # if time.time() - initial > 20:
            #     return None
            current_f, current = heapq.heappop(open_set) 

            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                # print(f"cost to goal position manhattan : {g_n[neighbor]}")
                return path

            closed_set.add(current)

            for dx , dy, dz in neighbor_cell: 
                neighbor = (
                    round(current[0] + dx * resolution, 2),
                    round(current[1] + dy * resolution, 2),
                    round(current[2] + dz * resolution, 2)
                )
                # print(neighbor)
                try:
                    node = self.tree.search(neighbor, 0)
                    if node is None:
                        continue
                    elif self.tree.isNodeOccupied(node):
                        continue

                    # if neighbor not in grid:
                    #     continue
                    if neighbor in closed_set:
                        continue
                    tentative_g = g_n[current] + math.sqrt(resolution**2 + resolution**2 + resolution**2)
                    # print(tentative_g)
                    if neighbor not in g_n or tentative_g < g_n[neighbor]:
                        came_from[neighbor] = current
                        g_n[neighbor] = tentative_g
                        f_n[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_n[neighbor], neighbor))
                except Exception as e:
                    print(e)

        return None
    
    def RRTstar(self):
        
        return None
    
    def list_to_pose_stamped(self, pos_list):
        poses = []
        if pos_list is not None:
            for pos in pos_list:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = float(pos[0])
                pose.pose.position.y = float(pos[1])
                pose.pose.position.z = float(pos[2])
                pose.pose.orientation.w = 1.0  # Assuming no rotation
                poses.append(pose)
            return poses
        else:
            print("no path to the goal position")

    
    def planner(self):
        trajectory = Path()
        trajectory.header.frame_id = "map"
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.poses = self.poses
        self.pub.publish(trajectory)


def main():
    rclpy.init(args=sys.argv)

    node = path_planner() 

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
