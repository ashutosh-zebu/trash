#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import numpy as np

from ament_index_python.packages import get_package_share_directory
import os

def create_pointcloud2(points, frame_id="map"):
    """
    Create a sensor_msgs.msg.PointCloud2 from Nx3 float32 points.
    """
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    header = Header()
    header.stamp = rclpy.clock.Clock().now().to_msg()
    header.frame_id = frame_id

    data = []
    for p in points:
        data.append(struct.pack('fff', *p))


    pointcloud_msg = PointCloud2()
    pointcloud_msg.header = header
    pointcloud_msg.height = 1
    pointcloud_msg.width = len(points)
    pointcloud_msg.fields = fields
    pointcloud_msg.is_bigendian = False
    pointcloud_msg.point_step = 12
    pointcloud_msg.row_step = pointcloud_msg.point_step * len(points)
    pointcloud_msg.is_dense = True
    pointcloud_msg.data = b''.join(data)

    return pointcloud_msg

class pcd(Node):
    def __init__(self):
        super().__init__('pcd_pub')
        self.get_logger().info("publisher initialized...") 

        pkg_share = get_package_share_directory('drone_nav_path_planner')
        path = os.path.join(pkg_share, 'pcd', 'voxelized.pcd')
        
        try:
            pcd = o3d.io.read_point_cloud(path)
            self.points = np.asarray(pcd.points)
            points = np.asarray(pcd.points)
            print(points.shape)
        except Exception as e:
            self.get_logger().error(f"Failed to load PCD file: {e}")
            return
        
        self.publisher_ = self.create_publisher(PointCloud2, 'pointcloud', 10)

        processing_rate = 0.1
        self.timer = self.create_timer(processing_rate, self.publisher)

    def publisher(self):
        msg = create_pointcloud2(self.points)
        self.publisher_.publish(msg)
        # self.get_logger().info("Published point cloud")

def main():
    rclpy.init(args=sys.argv)

    node = pcd() 

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
