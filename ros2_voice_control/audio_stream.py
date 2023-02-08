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

import pyaudio
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AudioStream(object):
    """
    TODO
    """
    def __init__(self):
        self.node = rclpy.create_node('AudioStream')
        self.node.publisher = self.node.create_publisher(String, 'audio_stream', 10)
        self.msg = String()

        # init pyaudio
        self.stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1,
                        rate=16000, input=True, frames_per_buffer=1024)
        
        self.pub_stream()


    def pub_stream(self):
        # start pocketsphinx and pyaudio
        self.stream.start_stream()

        while rclpy.ok():
            buf = self.stream.read(1024)
            if buf:
                self.node.publisher.publish(buf)
            else:
                break


def main(args=None):
    rclpy.init(args=args)

    a_stream = AudioStream()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    a_stream.node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()