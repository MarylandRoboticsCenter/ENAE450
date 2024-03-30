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


class CustomSubscriber(Node):

    def __init__(self):
        super().__init__('sub_searching')
        self.sub_1 = self.create_subscription(SingleArray, 'topic_1', self.topic_1_callback,10)
        self.sub_1  # prevent unused variable warning

        self.sub_2 = self.create_subscription(SingleArray, 'topic_2', self.topic_2_callback,10)
        self.sub_2  # prevent unused variable warning

        self.proc_msg = 0
        self.list_1 = [0]
        self.list_2 = [0]


    def topic_1_callback(self, msg):
        self.list_1 = msg.data.tolist()
        if (self.list_1[0] == self.list_2[0]):
            self.data_proc()

    def topic_2_callback(self, msg):
        self.list_2 = msg.data.tolist()
        if (self.list_1[0] == self.list_2[0]):
            self.data_proc()


    # data processing
    def data_proc(self):
        self.proc_msg = self.list_1[0]
        full_list = self.list_1[1:]
        int_to_search = self.list_2[1]

        self.get_logger().info('Serial number of the message is %s' % self.proc_msg)
        self.get_logger().info('Number to search is %s' % int_to_search)
        try:
            full_list.index(int_to_search)
            self.get_logger().info('Number was found: TRUE')
        except ValueError:
            self.get_logger().info('Number was found: FALSE')       


def main(args=None):
    rclpy.init(args=args)

    hw_sub = CustomSubscriber()

    rclpy.spin(hw_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hw_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
