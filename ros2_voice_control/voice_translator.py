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
import argparse

from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String


class VoiceTranslator(object):

    def __init__(self, pub_):
        self.speed = 0.4
        self.msg = Twist()
        self.node = rclpy.create_node('Voice_Translator')
        self.node.subscription = self.node.create_subscription(
            String,
            'voice_commands',
            self.listener_callback,
            10)
        self.node.subscription  # prevent unused variable warning
        self.node.publisher = self.node.create_publisher(
            Twist,
            pub_,
            10)
        self.node.timer = self.node.create_timer(0.5, self.timer_callback)


    def timer_callback(self):

        self.node.publisher.publish(self.msg)


    def listener_callback(self, msg):
        self.node.get_logger().info(f'I heard: "{msg.data}"')
        msg.data = msg.data.lower()

        if "full speed" in msg.data:
            if self.speed == 0.2:
                self.msg.linear.x = self.msg.linear.x*2
                self.msg.angular.z = self.msg.angular.z*2
                self.speed = 0.4
        if "half speed" in msg.data:
            if self.speed == 0.4:
                self.msg.linear.x = self.msg.linear.x/2
                self.msg.angular.z = self.msg.angular.z/2
                self.speed = 0.2
        if "forward" in msg.data:
            self.msg.linear.x = self.speed
            self.msg.angular.z = 0.0
        elif "left" in msg.data:
            if self.msg.linear.x != 0:
                if self.msg.angular.z < self.speed:
                    self.msg.angular.z += 0.05
            else:
                self.msg.angular.z = self.speed*2
        elif "right" in msg.data:
            if self.msg.linear.x != 0:
                if self.msg.angular.z > -self.speed:
                    self.msg.angular.z -= 0.05
            else:
                self.msg.angular.z = -self.speed*2
        elif "back" in msg.data:
            if self.msg.linear.x > 0:
                self.msg.linear.x = -self.speed
                self.msg.angular.z = 0.0
        elif "stop" in msg.data:
            self.msg = Twist()



def main(args=None):
    
    parser = argparse.ArgumentParser(
        description='Translates voice commands to turtlenot instructions')
    parser.add_argument('--pub', type=str,
        default='turtle1/cmd_vel',
        help='''ROS publisher destination
        (default: turtle1/cmd_vel)''')  # old mobile_base/commands/velocity
    args = parser.parse_args()
    rclpy.init(args=args)

    relay = VoiceTranslator(args.pub)

    rclpy.spin(relay.node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    relay.node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
