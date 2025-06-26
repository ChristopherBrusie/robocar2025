import serial
import time
from rclpy.node import Node
from std_msgs.msg import String


ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # Wait for ESP32 to reset

while True:


    # send speed input
    speed = input("Enter motor speed (0–100): ")
    if speed.isdigit():
        ser.write((speed + '\n').encode())
    else:
        print("Please enter a valid number.")
