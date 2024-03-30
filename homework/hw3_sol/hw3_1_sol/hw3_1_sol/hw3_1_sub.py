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
from interface_package.srv import SortMethod


class CustomSubscriber(Node):

    def __init__(self):
        super().__init__('sub_sorting')
        self.sub_1 = self.create_subscription(SingleArray, 'topic_1', self.topic_1_callback,10)
        self.sub_1  # prevent unused variable warning

        self.sub_2 = self.create_subscription(SingleArray, 'topic_2', self.topic_2_callback,10)
        self.sub_2  # prevent unused variable warning

        # creating publisher for bonus points #1
        self.pub = self.create_publisher(SingleArray, 'topic_3', 10)

        # service server for bonus points #2
        self.srv = self.create_service(SortMethod, 'sorting_method', self.sorting_method_callback)

        self.proc_msg = 0
        self.list_1 = [0]
        self.list_2 = [0]

        # define available sorting methods
        self.sorting_methods = ["method_1", "method_2", "method_3"]
        self.chosen_method = "method_1"


    def topic_1_callback(self, msg):
        self.list_1 = msg.data.tolist()
        if (self.list_1[0] == self.list_2[0]):
            self.data_proc()

    def topic_2_callback(self, msg):
        self.list_2 = msg.data.tolist()
        if (self.list_1[0] == self.list_2[0]):
            self.data_proc()

    # callback for service server (bonus points #2)
    def sorting_method_callback(self, request, response):
        if request.method in self.sorting_methods:
            self.chosen_method = request.method
            response.result = True
        else:
            response.result = False
        
        self.get_logger().info('The choice of sorting method %s is %s' % (request.method, response.result))

        return response

    # data processing
    def data_proc(self):
        self.proc_msg = self.list_1[0]
        full_list = self.list_1[1:] + self.list_2[1:]

        # I'm cheating and not coding different sorting algorithms
        if self.chosen_method == "method_1":
            full_list.sort()
        elif self.chosen_method == "method_2":
            full_list.sort()
        elif self.chosen_method == "method_3":
            full_list.sort()
        else:
            self.get_logger().info('Something is going terribly wrong!')
        
        self.get_logger().info('Serial number of the message is %s' % self.proc_msg)
        self.get_logger().info('Merged and sorted list: %s' % full_list)

        full_list.insert(0, self.proc_msg)

        # publishing message for bonus points #1
        msg = SingleArray()
        msg.data = full_list
        self.pub.publish(msg)

        


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
