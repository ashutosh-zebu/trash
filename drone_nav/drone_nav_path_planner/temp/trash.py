#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys


class pcd(Node):
    def __init__(self):
        super().__init__('pcd_pub')
        self.get_logger().info("publisher initialized...")  
        
        processing_rate = 0.1
        self.timer = self.create_timer(processing_rate, self.publisher)

    def publisher(self):
        pass

def main():
    rclpy.init(args=sys.argv)

    node = pcd() 

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
