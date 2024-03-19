# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import Twist


class DemoPublisher(Node):

    def __init__(self):
        super().__init__('lab6_pub')
        self.publisher_ = self.create_publisher(Twist, 'turtlesim1/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        lin_vel_descriptor = ParameterDescriptor(description='This is linear velocity of turtlesim')
        self.declare_parameter('lin_vel', 0.5, lin_vel_descriptor)


    def timer_callback(self):
        lin_vel = self.get_parameter('lin_vel').get_parameter_value().double_value
        # lin_vel = self.get_parameter('lab6_lin_vel').get_parameter_value()
        print(lin_vel)
        msg = Twist()
        msg.linear.x = lin_vel
        msg.angular.z = 0.5
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    lab6_publisher = DemoPublisher()

    rclpy.spin(lab6_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lab6_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
