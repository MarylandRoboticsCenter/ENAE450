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

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import random
from rclpy.node import Node

from interface_package.msg import SingleArray
from interface_package.srv import IntToSearch

class CustomPublisher(Node):

    def __init__(self):
        super().__init__('pub_generating')

        self.cb_group = MutuallyExclusiveCallbackGroup()
        self.tb_group = MutuallyExclusiveCallbackGroup()

        # setting up publishers
        self.pub_1 = self.create_publisher(SingleArray, 'topic_1', 10)

        # service client (bonus points)
        self.cli = self.create_client(IntToSearch, 'int_to_search')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        # setting up timer
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=self.tb_group)
        self.i = 1

    def timer_callback(self):

        list_1 = random.sample(range(0, 100), 21)
        list_2 = random.sample(range(1, 100), 1)

        list_1.insert(0, self.i)
        
        # sending topic message
        msg_1 = SingleArray()
        msg_1.data = list_1
        self.pub_1.publish(msg_1)        

        # calling service client (bonus points)
        req = IntToSearch.Request()
        req.msg_num = self.i
        req.int_search = list_2[0]
        res = self.cli.call(req)
        
        if res.result:
            self.get_logger().info( 'Result of number search is SUCCESS')
        else:
            self.get_logger().info( 'Result of number search is FAIL')


        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    hw_pub_bonus = CustomPublisher()

    executor = MultiThreadedExecutor()
    executor.add_node(hw_pub_bonus)    
    executor.spin()

    rclpy.spin(hw_pub_bonus)

    # Destroy the node explicitly
    hw_pub_bonus.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
