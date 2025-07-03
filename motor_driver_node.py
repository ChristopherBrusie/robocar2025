#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


import serial
import time
import math
import numpy as np


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        self.get_logger().info('Motor Driver Node has been started.')

        self.subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        time.sleep(2)  # Wait for ESP32 to reset

        # wheel stuff
        rpm_factor = 60 / (math.pi * 0.06) # wheel diameter 0.06m



        # Set up a timer or subscriber to handle motor speed commands
        # self.timer = self.create_timer(1.0, self.timer_callback)



    def cmd_vel_callback(self, msg):
        # motor order, direction 
        #       ^
        # 0   O| |O   3
        # 1   O| |O   2
        # 

        twist_vec = np.array([msg.linear.x, msg.linear.y, msg.angular.z]) # the car cannot fly yet
        rpm_vec = twist_vec * self.rpm_factor












def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

