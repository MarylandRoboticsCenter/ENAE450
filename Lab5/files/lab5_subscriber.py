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

from interface_package.msg import SingleArray
from interface_package.srv import AddThreeInts

class DemoSubscriber(Node):

    def __init__(self):
        super().__init__('lab5_sub')

        self.srv = self.create_service(AddThreeInts, 'add_three_ints', self.add_three_ints_callback)
        self.srv = self.create_service(AddThreeInts, 'add_three_ints_and_mult', self.add_three_ints_mult_callback)

        self.declare_parameter('lab5_param', 1)

        self.subscription = self.create_subscription(SingleArray, 'topic', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def add_three_ints_callback(self, request, response):
        response.sum = request.a + request.b + request.c
        self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c))

        return response
    
    def add_three_ints_mult_callback(self, request, response):
        lab5_param = self.get_parameter('lab5_param').get_parameter_value().integer_value
        response.sum = (request.a + request.b + request.c)*lab5_param
        self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c))

        return response    

    def listener_callback(self, msg):
        lab5_param = self.get_parameter('lab5_param').get_parameter_value().integer_value

        data_arr = msg.data.tolist()
        new_arr = [x + lab5_param for x in data_arr]
        
        self.get_logger().info('The original array: %s' % data_arr)
        self.get_logger().info('Parameter value: %s' % lab5_param)
        self.get_logger().info('The original array: %s' % new_arr)


def main(args=None):
    rclpy.init(args=args)

    lab5_subscriber = DemoSubscriber()

    rclpy.spin(lab5_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lab5_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
