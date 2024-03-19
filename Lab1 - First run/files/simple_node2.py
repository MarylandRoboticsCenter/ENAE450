#!/usr/bin/env python3

import rclpy
from rclpy.node import Node



def main(args=None):
    rclpy.init(args=args) #initialize
    node = Node("py_test")
    node.get_logger().info("Hello ROS2")
    rclpy.spin(node)
    
    rclpy.shutdown() #node shutdown


if __name__ == "__main__":
    main()
