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
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pocketsphinx import *


class SphinxDecoder(object):
    """
    VoiceDetector uses pyaudio and pocketsphinx to listen for live input and supports
    custom language models, command lists, and topics to publish to.
    """
    def __init__(self, model, dic, gram):
        self.node = rclpy.create_node('Sphinx_Decoder')
        self.node.publisher_ = self.node.create_publisher(String, 'voice_commands', 10)
        self.node.subscriber_ = self.node.create_subscription(String, 'audio_stream', self.decode_audio, 10)
        self.msg = String()

        # init pocketsphinx
        config = Config(hmm=model, dict=dic, jsgf=gram)
        self.decoder = Decoder(config)

    def decode_audio(self, data):
        # start pocketsphinx
        self.decoder.start_utt()
        
        self.decoder.process_raw(data.data, False, False)

        hypothesis = self.decoder.hyp()
        if hypothesis != None:
            print(hypothesis.hypstr)
            self.decoder.end_utt()
        #    self.msg.data = hypothesis.hypstr
        #    self.node.publisher_.publish(self.msg)
            self.node.get_logger().info('Publishing: "%s"' % hypothesis.hypstr)
            self.decoder.start_utt()


def main(args=None):
    rclpy.init(args=args)

    # get current directory for kwlist and dic
    current_dir = os.path.dirname(os.path.realpath(__file__))
    current_dir = current_dir + "/../../../../share/ros2_voice_control/"

    # look for custom model, dict, kwlist, and publisher location
    parser = argparse.ArgumentParser(
        description='Control ROS turtlebot using pocketsphinx.')
    parser.add_argument('--hmm', type=str,
        default=get_model_path('en-us')+'/en-us',
        help='''acoustic model path
        (default: en-us''')
    parser.add_argument('--dic', type=str,
        default=current_dir+'vc_cmd.dic',
        help='''PocketSphinx dictionary file
        (default: vc_cmd.dic)''')
    parser.add_argument('--gram', type=str,
        default=current_dir+'vc_cmd.gram',
        help='''PocketSphinx grammar file
        (default: vc_cmd.gram)''')
    
    args = parser.parse_args()

    voice_decoder = SphinxDecoder(args.hmm, args.dic, args.gram)

    rclpy.spin(voice_decoder.node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    voice_decoder.node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()