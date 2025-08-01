#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
import sys
import octomap
import numpy as np
from drone_nav_path_planner.srv import Pos
import math

def points_are_close(p1, p2, tol=1e-3):
    return all(math.isclose(a, b, abs_tol=tol) for a, b in zip(p1, p2))

class localplanner(Node):
    def __init__(self):
        super().__init__('local_planner')
        
        self.goal_position = []
        self.orientation = []
        self.goal_pos = [20.1, 8.9, 8.9]
        self.tree_resolution = 0.2

        self.trajectory_sub = self.create_subscription(Path, "/nav_trajectory",self.trajectory_callback, 10)
        self.local_obstacle_sub = self.create_subscription(MarkerArray, "/local_occupancy_grid", self.voxel, 10)

        # self.goal_pose = self.create_subscription(PoseStamped,"/goal_pose",self.position, 10)
        self.planner_client = self.create_client(Pos,"/trajectory_planner")


        while not self.planner_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('planner service not available, waiting again...')

    # def position(self, msg):
    #     # print(msg.pose.position)
    #     # print(msg.pose.orientation)

    #     self.goal_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    #     self.orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]


    def trajectory_callback(self, msg):
        self.trajectory_positions = []
        for pose_stamped in msg.poses:
            pt = pose_stamped.pose.position
            qt = pose_stamped.pose.orientation

            self.trajectory_positions.append((pt.x, pt.y, pt.z))
            # self.trajectory_orientations.append((qt.x, qt.y, qt.z, qt.w))
        

    def voxel(self, msg):
        try:
            self.obstacle = []
            self.tree = octomap.OcTree(self.tree_resolution)

            for marker in msg.markers:
                for point in marker.points:
                    # print(f"Point: {point}")

                    self.obstacle.append((point.x, point.y, point.z))
                    
            self.tree.insertPointCloud(
                np.array(self.obstacle), np.array((0.0, 0.0, 0.0))
            )
            for path in self.trajectory_positions:
                for node in self.tree.begin_leafs():
                    if self.tree.isNodeOccupied(node):

                        x, y, z = self.tree.keyToCoord(node.getKey())
                        voxel = [x, y ,z]
                        if points_are_close(path, voxel):
                            print(f"⚠ Obstacle near path point: {path} matches voxel {voxel}")
                        else:
                            print(f"Ea")
                        # break
                        # print(f"Occupied voxel at: x={x}, y={y}, z={z}")

            # for path in self.trajectory_positions:
            #     print(path)

            #     node = self.tree.search(path, 0)
            #     # print(node)
            #     # if node is None:
            #     #     continue
            #     try:
            #         if self.tree.isNodeOccupied(node):
            #             print("obstacle in the path")
            #             self.planner_service()
            #     except Exception as e:
            #         print(e)
            # #         # self.planner_service()
            # #         # break
            # #     else:
            # #         print("path is free")
            # # print(len(self.obstacle))
        except Exception as e:
            print(e)


    def planner_service(self):
        '''
            Description:    payload_service function is used to perform payload operation.
        '''
        if Pos is None:
            return
        req = Pos.Request()
        req.state = True
        req.goal_position.x = self.goal_pos[0]
        req.goal_position.y = self.goal_pos[1]
        req.goal_position.y = self.goal_pos[2]

        self.planner_client.call_async(req)

def main():
    rclpy.init(args=sys.argv)

    node = localplanner() 

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()



