#!/usr/bin/env python3

import rclpy
import sys
from rclpy.node import Node
from pymavlink import mavutil
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import numpy as np

import math

class Drone(Node):
    def __init__(self):
        super().__init__('controller')
        self.get_logger().info("controller initialized...")

        self.master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.master.wait_heartbeat()

        self.points = None

        self.index = 0
        self.nth = None
        # print(self.nth)
        self.current_pos = None
        self.path_publisher = self.create_publisher(Path, "/nav_current_trajectory", 10)
        self.path_subscriber = self.create_subscription(Path, "/nav_trajectory", self.path_callback, 10)

        self.timer1 = self.create_timer(0.001, self.process)
        self.timer2 = self.create_timer(0.05,self.collectdata)

    # -------------------------------------------------------------------------------------------------------------------------------------------------------------
    def arm(self):
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0, 1, 0, 0, 0, 0, 0, 0)
        self.master.motors_armed_wait()
        return True
    
    def set_takeoff(self,target_altitude):
        # self.master.mav.set_attitude_target_send(
        #                             0,  # system time in milliseconds
        #                             self.settings.target_system,  # target system
        #                             0,            # target component
        #                             mask,         # type mask
        #                             att_target,   # quaternion attitude
        #                             radians(roll_rate),    # body roll rate
        #                             radians(pitch_rate),   # body pitch rate
        #                             radians(yaw_rate),     # body yaw rate
        #                             thrust)       # thrust
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0, 0, 0, 0, 0, 0, 0, target_altitude)

    def path_callback(self,msg):
        if self.points is None:
            self.points = []
            
            for pose_stamped in msg.poses:
                pos = pose_stamped.pose.position
                self.points.append([round(pos.x,2), round(pos.y,2), round(pos.z,2)])
        

    def SET_POSITION_TARGET_LOCAL_NED(self, coordinate_frame, mask, x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_rate):
        # https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#copter-commands-in-guided-mode-set-position-target-local-ned
        # https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED

        # note:
        # Use Position : 0b110111111000 / 0x0DF8 / 3576 (decimal)
        # Use Velocity : 0b110111000111 / 0x0DC7 / 3527 (decimal)
        # Use Acceleration : 0b110000111111 / 0x0C3F / 3135 (decimal)
        # Use Pos+Vel : 0b110111000000 / 0x0DC0 / 3520 (decimal)
        # Use Pos+Vel+Accel : 0b110000000000 / 0x0C00 / 3072 (decimal)
        # Use Yaw : 0b100111111111 / 0x09FF / 2559 (decimal)
        # Use Yaw Rate : 0b010111111111 / 0x05FF / 1535 (decimal)

        if mask == 0:
            type_mask = 0b001111111000  # for yaw
        if mask == 1:
            type_mask = 0b111111111000  # for position
        if mask == 2:
            type_mask = 0b011111000111  # for velocity 
        if mask == 3:
            type_mask = 0b111000111000  # for acceleration
        if mask == 4:
            type_mask = 0b101111000000
        
                        
        self.master.mav.set_position_target_local_ned_send(
                0, # timestamp
                self.master.target_system,                               # target system_id
                self.master.target_component,                            # target component id
                coordinate_frame,          # coordinate_frame
                type_mask ,# 0b101111111000,                                 # mask specifying use-only-x-y-z-yaw
                x,                                                  # x
                y,                                                  # y
                -z,                                                 # z
                vx,                                                 # X velocity in m/s (positive is forward or North)
                vy,                                                 # Y velocity in m/s (positive is right or East)
                vz,                                                 # Z velocity in m/s (positive is down)
                ax,                                                 # X acceleration in m/s/s (positive is forward or North)
                ay,                                                 # Y acceleration in m/s/s (positive is right or East)
                az,                                                 # Z acceleration in m/s/s (positive is down)
                yaw,                                                # yaw or heading in radians (0 is forward or North)
                yaw_rate                                            # yaw rate in rad/s
                )

        # 0b 0 0 0 0 0 0 0 0 0 0 0 0 
        # Init Yaw_Rate Yaw NC AZ AY AX VZ VY VX PZ PY PX
        # 0 = ON; 1 = OFF

    
    # -------------------------------------------------------------------------------------------------------------------------------------------------------------

    def collectdata(self):
        msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        self.current_pos = [round(msg.x,2),round(msg.y,2),round(msg.z,2)]
        self.current_alt = msg.z
        # _,_,yaw = euler_from_quaternion()
        # print(self.current_pos)
        # print(f"NED Position -> X: {msg.x:.2f} m, Y: {msg.y:.2f} m, Z (alt): {msg.z:.2f} m")

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

    def process(self):
        try:
            if self.points is not None and self.current_pos is not None:
                self.nth = len(self.points)
                
                if self.index == self.nth:
                    return
                
                # print(self.points)
                target = self.points[self.index]
                self.SET_POSITION_TARGET_LOCAL_NED(coordinate_frame= mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                        mask = 1, 
                                                        x = target[0], 
                                                        y = -target[1], 
                                                        z = target[2], 
                                                        vx = 0.0, 
                                                        vy = 0.0, 
                                                        vz = 0.0, 
                                                        ax = 0, 
                                                        ay = 0, 
                                                        az = 0, 
                                                        yaw = 0.0,
                                                        yaw_rate=0.0
                                                        )
                threshold = 0.1
                print(f"Current Pos: {self.current_pos}")
                print(f"Current Target: {self.points[self.index]}")
                error = [0, 0, 0]
                error[0] = np.array(self.current_pos[0])- np.array(self.points[self.index][0])
                error[1] = np.array(self.current_pos[1])- np.array(-self.points[self.index][1])
                error[2] = np.array(self.current_pos[2])- np.array(-self.points[self.index][2])
                # print(self.index)
                # print(self.points)

                # error = np.array(self.current_pos) - np.array(self.points[self.index])
                # error = np.linalg.norm(error)
                print(error)
                if(np.linalg.norm(error) < threshold):
                    # del self.points[0]
                    self.points.pop(self.index)
                    self.poses = self.list_to_pose_stamped(self.points)
                    trajectory = Path()
                    trajectory.header.frame_id = "map"
                    trajectory.header.stamp = self.get_clock().now().to_msg()
                    trajectory.poses = self.poses
                    self.path_publisher.publish(trajectory)
                    print(f" reached the position : {self.points[self.index]}")
                    # self.index = self.index + 1
        except Exception as e:
            raise e

def main():
    rclpy.init(args=sys.argv)

    node = Drone() 

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()