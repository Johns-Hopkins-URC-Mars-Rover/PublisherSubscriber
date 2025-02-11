
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

class Publisher(Node):

    def __init__(self, name:str, topic:str):
        super().__init__(name)
        self.publisher_ = self.create_publisher(String, topic, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_data)

    def publish_data(self):
        msg = String()
        msg.data = self.data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def update(self, data:dict):
        self.data = str(data)


def main(args=None):
    rclpy.init(args=args)

    x = {'a' : 1, 'b' : 2, 'c' : 3}

    publisher = Publisher('sample_subscriber', 'topic')
    executor = rclpy.get_global_executor()
    try:
        executor.add_node(publisher)
        while executor.context.ok():
            publisher.update(x)
            executor.spin_once()
    finally:
        executor.remove_node(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
