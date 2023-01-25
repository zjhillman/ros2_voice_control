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

import argparse
import pyaudio
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pocketsphinx import *


class VoiceDetector(object):
    """
    VoiceDetector uses pyaudio and pocketsphinx to listen for live input and supports
    custom language models, command lists, and topics to publish to.
    """
    def __init__(self, model, lexicon, kwlist, pub):
        self.node = rclpy.create_node('Voice_Detector')
        self.node.publisher_ = self.node.create_publisher(String, 'voice_commands', 10)
        self.msg = String()

        # init pocketsphinx
        config = Config(hmm = model, dict = lexicon, kws = kwlist)
        self.decoder = Decoder(config)

        # init pyaudio
        self.stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1,
                        rate=16000, input=True, frames_per_buffer=1024)

    def decode_voice(self):
        # start pocketsphinx and pyaudio
        self.stream.start_stream()
        self.decoder.start_utt()

        while rclpy.ok():
            buf = self.stream.read(1024)
            if buf:
                self.decoder.process_raw(buf, False, False)
            else:
                break

            hypothesis = self.decoder.hyp()
            if hypothesis != None:
                print(f"\n{hypothesis.hypstr}\n")
                self.decoder.end_utt()
                self.msg.data = hypothesis.hypstr
                self.node.publisher_.publish(self.msg)
                self.node.get_logger().info('Publishing: "%s"' % self.msg)
                self.decoder.start_utt()

def main(args=None):
    rclpy.init(args=args)

    # get current directory for kwlist and dic
    current_dir = os.path.dirname(os.path.realpath(__file__))
    current_dir = current_dir + "/../../../../share/ros2_voice_control/"

    # look for custom model, dict, kwlist, and publisher location
    parser = argparse.ArgumentParser(
        description='Control ROS turtlebot using pocketsphinx.')
    parser.add_argument('--model', type=str,
        default=get_model_path('en-us')  +  '/en-us',
        help='''acoustic model path
        (default: en-us''')
    parser.add_argument('--lexicon', type=str,
        default=current_dir+'voice_cmd.dic',
        help='''pronunciation dictionary
        (default: voice_cmd.dic)''')
    parser.add_argument('--kwlist', type=str,
        default=current_dir+'voice_cmd.kwlist',
        help='''keyword list with thresholds
        (default: voice_cmd.kwlist)''')
    parser.add_argument('--pub', type=str,
        default='turtle1/cmd_vel',
        help='''ROS publisher destination
        (default: turtle1/cmd_vel)''')  # old mobile_base/commands/velocity
    args = parser.parse_args()

    voice_detector = VoiceDetector(args.model, args.lexicon, args.kwlist, args.pub)

    voice_detector.decode_voice()
    #rclpy.spin(voice_detector.node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    voice_detector.node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()