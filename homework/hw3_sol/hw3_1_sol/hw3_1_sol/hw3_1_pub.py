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

import random
import rclpy
from rclpy.node import Node

from interface_package.msg import SingleArray
from interface_package.srv import SortMethod

class CustomPublisher(Node):

    def __init__(self):
        super().__init__('pub_generating')
        # setting up publishers
        self.pub_1 = self.create_publisher(SingleArray, 'topic_1', 10)
        self.pub_2 = self.create_publisher(SingleArray, 'topic_2', 10)

        # service client (bonus points #2)
        self.cli = self.create_client(SortMethod, 'sorting_method')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        req = SortMethod.Request()
        req.method = 'method_2'
        
        future = self.cli.call_async(req) # call service (bonus points #2)
        rclpy.spin_until_future_complete(self, future) # wait until service call is complete (bonus points #2)

        res = future.result()
        self.get_logger().info( 'Result of sorting method choice is %s' % res.result)

        # setting up timer
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 1

    def timer_callback(self):

        list_1 = random.sample(range(0, 100), 10)
        list_2 = random.sample(range(0, 100), 10)

        list_1.insert(0, self.i)
        list_2.insert(0, self.i)

        msg_1 = SingleArray()
        msg_2 = SingleArray()

        msg_1.data = list_1
        msg_2.data = list_2        

        self.pub_1.publish(msg_1)
        self.pub_2.publish(msg_2)

        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    hw_pub = CustomPublisher()

    rclpy.spin(hw_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hw_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
