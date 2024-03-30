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
from interface_package.srv import IntToSearch

class CustomSubscriber(Node):

    def __init__(self):
        super().__init__('sub_searching')
        self.sub_1 = self.create_subscription(SingleArray, 'topic_1', self.topic_1_callback,10)
        self.sub_1  # prevent unused variable warning

        # service server for bonus points
        self.srv = self.create_service(IntToSearch, 'int_to_search', self.int_search_callback)

        self.proc_msg = 0
        self.list_1 = [0]
        self.list_2 = [0]

    # topic callback
    def topic_1_callback(self, msg):
        self.list_1 = msg.data.tolist()


    # callback for service server (bonus points #2), includes data processing
    def int_search_callback(self, request, response):
        
        if (self.list_1[0] == request.msg_num):
            self.proc_msg = request.msg_num

            full_list = self.list_1[1:]
            int_to_search = request.int_search

            self.get_logger().info('Serial number of the message is %s' % self.proc_msg)
            self.get_logger().info('Number to search is %s' % int_to_search)

            try:
                full_list.index(int_to_search)
                self.get_logger().info('Number was found: TRUE')
                response.result = True
            except ValueError:
                self.get_logger().info('Number was found: FALSE')
                response.result = False

        return response


def main(args=None):
    rclpy.init(args=args)

    hw_sub = CustomSubscriber()

    rclpy.spin(hw_sub)

    # Destroy the node explicitly
    hw_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
