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
import os
import colorama
from colorama import Fore

from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String

DEBUG = False


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
        
        current_dir = os.path.dirname(os.path.realpath(__file__))
        current_dir = current_dir + "/../../../../share/ros2_voice_control/"
        corpus = open(current_dir + 'corpus.txt', 'r')
        self.vc_cmd = corpus.read().splitlines()
        if DEBUG:
            print(self.vc_cmd)


    def listener_callback(self, msg):
        if DEBUG:
            self.node.get_logger().info(f'I heard: "{msg.data}"')
        print('\nYou said: ' + Fore.CYAN + msg.data.lower() + Fore.RESET)

        if msg.data[:-1] in self.vc_cmd:
            self.process_command(msg.data[:-1])


    def process_command(self, cmd):
        if cmd == 'MOVE FORWARD':
            self.msg.linear.x = self.speed
            self.msg.angular.z = 0.0
        elif cmd == 'MOVE BACKWARD':
            self.msg.linear.x = -self.speed
            self.msg.angular.z = 0.0
        elif cmd == 'TURN LEFT':
            if self.msg.linear.x != 0.0:
                if self.msg.angular.z < self.speed:
                    self.msg.angular.z += 0.05
            else:
                self.msg.angular.z = self.speed*2
        elif cmd == 'TURN RIGHT':
            if self.msg.linear.x != 0.0:
                if self.msg.angular.z > -self.speed:
                    self.msg.angular.z -= 0.05
            else:
                self.msg.angular.z = -self.speed*2
        elif cmd == 'FULL SPEED':
            if self.speed == 0.2:
                self.msg.linear.x = self.msg.linear.x*2
                self.msg.angular.z = self.msg.angular.z*2
                self.speed = 0.4
        elif cmd == 'HALF SPEED':
            if self.speed == 0.4:
                self.msg.linear.x = self.msg.linear.x/2
                self.msg.angular.z = self.msg.angular.z/2
                self.speed = 0.2
        elif cmd == 'STOP' or cmd == 'HALT':
            self.msg = Twist()
        elif cmd == 'CAN YOU BRING ME A COOKIE':
            self.turtlebot_say('Got it')
        elif cmd == 'CAN YOU BRING ME A BISCUIT':
            self.turtlebot_say('What do you mean by "biscuit"?')
        elif cmd == 'CAN YOU THROW THIS PAPER IN THE TRASH':
            self.turtlebot_say('Got it')
        elif cmd == 'CAN YOU THROW THIS PAPER IN THE BIN':
            self.turtlebot_say('What do you mean by "bin"?')
        elif cmd == 'CAN YOU THROW THIS IN THE TRASH':
            self.turtlebot_say('What do you mean by "this"?')
        else:
            self.msg = Twist()
            print('ERROR: TODO')


    def turtlebot_say(self, response):
        self.msg = Twist()
        print('\n' + Fore.RED + response + Fore.RESET)


    def timer_callback(self):
        self.node.publisher.publish(self.msg)

def main(args=None):
    
    # parse for arguments
    parser = argparse.ArgumentParser(
        description='Translates voice commands to turtlenot instructions')
    parser.add_argument('--pub', type=str,
        default='turtle1/cmd_vel',
        help='''ROS publisher destination
        (default: turtle1/cmd_vel)''')  # old mobile_base/commands/velocity
    args = parser.parse_args()

    rclpy.init(args=None)

    relay = VoiceTranslator(args.pub)

    rclpy.spin(relay.node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    relay.node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
