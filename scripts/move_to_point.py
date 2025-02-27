"""
"""
from pathlib import Path
import os
from time import sleep
import argparse

import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

WORLD = "car_world"
CAR_ID = 4 # important to set the car position
cone_model = Path(os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")) / "cone.sdf"
assert cone_model.exists(), "Cone model does not exist"

class TargetPointPublisher(Node):
    def __init__(self, x: float | int, y: float | int):
        super().__init__("TargetPointPublisher") # "NodeName" will be displayed in rqt_graph
        self.publisher = self.create_publisher(PoseStamped, "/target_point", 10)
        self.get_logger().info(f"Publishing point: {x, y}")
        t = self.get_clock().now()
        msg = PoseStamped()
        msg.header.stamp = t.to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = x
        msg.pose.position.y = y
        self.publisher.publish(msg)



def main():
    parser = argparse.ArgumentParser(description="Publish a single x,y coordinate to the /target_point topic")
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    args = parser.parse_args()
    rclpy.init()
    TargetPointPublisher(x=args.x, y=args.y)

if __name__ == "__main__":
    main()
