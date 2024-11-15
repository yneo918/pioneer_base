import rclpy
from rclpy.node import Node
import serial
import time
from sensor_msgs.msg import Range


import numpy as np
import re


class AgTruthDistance(Node):
    def __init__(self):
        super().__init__("tf03_distance")

        self.ser = serial.Serial(
            port="/dev/ttyACM0",
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0,
        )

        self.pub_d1 = self.create_publisher(
            Range, "/r4/distance/front", 10
        )
        timer_period = 0.1
        self.distance_timer = self.create_timer(timer_period, self.timer_callback)

        self.distance_prev = [0, 0]

    def get_serial_distances(self) -> list:
        distance_string = str(self.ser.readline())
        pattern = r"\b\d+\b"

        distances = re.findall(pattern, distance_string)
        if len(distances) == 0:
            return [self.distance_prev[0], self.distance_prev[1]]
        distances = [int(dist) for dist in distances]

        return distances

    def timer_callback(self) -> None:
        d1 = Range()
        
        
        distances = self.get_serial_distances()
        self.distance_prev = distances

        d1.header.frame_id = "front tof sensor"
        #d1.header.stamp.nanosec = time.monotonic_ns()
        d1.radiation_type = 2
        d1.min_range = 0.5
        d1.max_range = 170.0
        d1.range = float(distances[0])/100.0

        self.pub_d1.publish(d1)


def main(args=None):
    rclpy.init(args=args)

    ag_vineyard_distance = AgTruthDistance()

    rclpy.spin(ag_vineyard_distance)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ag_vineyard_distance.destroy_node()
    rclpy.shutdown()
