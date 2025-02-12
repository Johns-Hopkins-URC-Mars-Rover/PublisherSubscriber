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

from std_msgs.msg import String


class Subscriber(Node):

    def __init__(self, node_name:str, topic:str, data):
        super().__init__(node_name)
        self.subscription = self.create_subscription(
            String, topic, self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.data = data

    def listener_callback(self, msg):
        #TODO: save data somewhere (wilson says list) ig we're doing function
        self.data(msg)
        self.get_logger().info('Recieved: "%s"' % msg.data)

def example_recieve_data(msg):
    # do something with data
    pass

def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber('sample_subscriber', 'topic', example_recieve_data)
    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
