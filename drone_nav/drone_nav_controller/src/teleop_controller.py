#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
from pymavlink import mavutil
from geometry_msgs.msg import Twist

class teleop(Node):
    def __init__(self):
        super().__init__('teleop_controller')
        self.get_logger().info("teleop initialized...")

        self.master = mavutil.mavlink_connection('udp:localhost:14552')     #  setup the connection with the drone through mavlink
        self.master.wait_heartbeat()   
        print("Heartbeat recieved")
        self.arm()
        self.set_takeoff(0.5)
        self.sub = self.create_subscription(Twist, 
                                            "/cmd_vel",
                                            self.callback,
                                            10
                                            )
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

    def callback(self, msg):
        vel_x = msg.linear.x
        yaw_rate = msg.angular.z
        vel_z = msg.linear.z
        # print(f"{vel_x}, {vel_y}, {vel_z}")
        self.SET_POSITION_TARGET_LOCAL_NED(coordinate_frame= mavutil.mavlink.MAV_FRAME_BODY_NED,
                                                mask = 2, 
                                                x = 0.0, 
                                                y = 0.0, 
                                                z = 0.0, 
                                                vx = vel_x, 
                                                vy = 0.0, 
                                                vz = -vel_z, 
                                                ax = 0, 
                                                ay = 0, 
                                                az = 0, 
                                                yaw = 0.0,
                                                yaw_rate=yaw_rate
                                                )
    
def main():
    rclpy.init(args=sys.argv)

    node = teleop() 

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()