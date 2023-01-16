#!/usr/bin/env python

"""This module is a simple demonstration of voice control
for ROS turtlebot using pocketsphinx
"""

import argparse

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from pocketsphinx import *
import pyaudio

class SphinxNode(Node):
    def __init__(self):
        super().__init__('voice_cmd_vel')
        self.get_logger().info('SphinxNode has been started')

class ASRControl(object):
    """Simple voice control interface for ROS turtlebot

    Attributes:
        model: model path
        lexicon: pronunciation dictionary
        kwlist: keyword list file
        pub: where to send commands (default: 'mobile_base/commands/velocity')

    """
    def __init__(self, model, lexicon, kwlist, pub):
        # initialize ROS
        self.speed = 0.2
        self.msg = Twist()

        self.node = SphinxNode()

        # you may need to change publisher destination depending on what you run
        self.node.pub = self.node.create_publisher(Twist, pub, 10)

        # initialize pocketsphinx
        config = Config(hmm=model, dict=lexicon, kws=kwlist)
        print(config.dumps())

        stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1,
                        rate=16000, input=True, frames_per_buffer=1024)
        stream.start_stream()

        self.decoder = Decoder(config)
        self.decoder.start_utt()

        while True:
            buf = stream.read(1024)
            if buf:
                self.decoder.process_raw(buf, False, False)
            else:
                rclpy.shutdown()
                break
            self.parse_asr_result()

    def parse_asr_result(self):
        """
        move the robot based on ASR hypothesis
        """
        if self.decoder.hyp() != None:
            for seg in self.decoder.seg():
                print ([(seg.word, seg.prob, seg.start_frame, seg.end_frame)])
                print ("Detected keyphrase, restarting search")
                seg.word = seg.word.lower()
                self.decoder.end_utt()
                self.decoder.start_utt()
                # you may want to modify the main logic here
                if seg.word.find("full speed") > -1:
                    if self.speed == 0.2:
                        self.msg.linear.x = self.msg.linear.x*2
                        self.msg.angular.z = self.msg.angular.z*2
                        self.speed = 0.4
                if seg.word.find("half speed") > -1:
                    if self.speed == 0.4:
                        self.msg.linear.x = self.msg.linear.x/2
                        self.msg.angular.z = self.msg.angular.z/2
                        self.speed = 0.2
                if seg.word.find("forward") > -1:
                    self.msg.linear.x = self.speed
                    self.msg.angular.z = 0.0
                elif seg.word.find("left") > -1:
                    if self.msg.linear.x != 0:
                        if self.msg.angular.z < self.speed:
                            self.msg.angular.z += 0.05
                    else:
                        self.msg.angular.z = self.speed*2
                elif seg.word.find("right") > -1:
                    if self.msg.linear.x != 0:
                        if self.msg.angular.z > -self.speed:
                            self.msg.angular.z -= 0.05
                    else:
                        self.msg.angular.z = -self.speed*2
                elif seg.word.find("back") > -1:
                    self.msg.linear.x = -self.speed
                    self.msg.angular.z = 0.0
                elif seg.word.find("stop") > -1 or seg.word.find("halt") > -1:
                    self.msg = Twist()

        self.node.pub.publish(self.msg)

def main():
    rclpy.init(args=None)
	
    parser = argparse.ArgumentParser(
        description='Control ROS turtlebot using pocketsphinx.')
    parser.add_argument('--model', type=str,
        default=get_model_path('en-us')  +  '/en-us',
        help='''acoustic model path
        (default: en-us''')
    parser.add_argument('--lexicon', type=str,
        default='voice_cmd.dic',
        help='''pronunciation dictionary
        (default: voice_cmd.dic)''')
    parser.add_argument('--kwlist', type=str,
        default='voice_cmd.kwlist',
        help='''keyword list with thresholds
        (default: voice_cmd.kwlist)''')
    parser.add_argument('--rospub', type=str,
        default='turtle1/cmd_vel',
        help='''ROS publisher destination
        (default: turtle1/cmd_vel)''')  # old mobile_base/commands/velocit
    args = parser.parse_args()

    ASRControl(args.model, args.lexicon, args.kwlist, args.rospub)
	

if __name__ == '__main__':
    main()
