#!/usr/bin/env python

"""This module is a simple demonstration of voice control
for ROS turtlebot using pocketsphinx
"""

import argparse
import pyaudio
import threading

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String

from pocketsphinx import *

class ASRControl(object):
    """Simple voice control interface for ROS turtlebot
    Attributes:
        model: model path
        lexicon: pronunciation dictionary
        kwlist: keyword list file
        pub: where to send commands (default: 'mobile_base/commands/velocity')
    """
    def __init__(self, model, lexicon, kwlist, pub):
        # initialize ASR Object
        self.speed = 0.2
        self.name = 'voice_cmd_vel'
        self.node = rclpy.create_node(self.name)
        self.node.get_logger().info(f'{self.name} has been started')

        # create publisher & subscriber
        self.node.pub_ = self.node.create_publisher(Twist, pub, 10)

        # create a rate object at 5Hz
        #self.rate = self.node.create_rate(5)

        # initialize Twist message and set to 'stop' by default
        self.msg = Twist()
        self.msg.linear.x=0.0
        self.msg.angular.z=0.0

        # initialize pocketsphinx
        config = Config(hmm=model, dict=lexicon, kws=kwlist)

        self.stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1,
                        rate=16000, input=True, frames_per_buffer=1024)

        self.decoder = Decoder(config)

    def decode_asr(self):
        self.stream.start_stream()
        self.decoder.start_utt()

        while rclpy.ok():
            buf = self.stream.read(1024)
            if buf:
                self.decoder.process_raw(buf, False, False)
            else:
                break
            self.parse_asr_result()

    def parse_asr_result(self):
        """
        move the robot based on ASR hypothesis
        """
        if self.decoder.hyp() != None:
            for seg in self.decoder.seg():
                print (f"\n[DECODER] Detected {seg.word}, {seg.prob}%, {seg.start_frame}, {seg.end_frame}")
                seg.word = seg.word.lower()
                self.decoder.end_utt()
                self.decoder.start_utt()
                print("[DECODER] new utterance started")
                print(f"[DECODER] Old msg: {self.msg}")
                print(f"[INFO] speed: {self.speed}")
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
                    if self.msg.linear.x > 0:
                        self.msg.linear.x = -self.speed
                        print("[LOGGER]")
                        self.msg.angular.z = 0.0
                elif seg.word.find("stop") > -1:
                    self.msg = Twist()
                print(f"[DECODER] New msg: {self.msg}")
            
        self.node.pub_.publish(self.msg)
        # self.node.get_logger().info("msg published")

        
        

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

    try:
        asr = ASRControl(args.model, args.lexicon, args.kwlist, args.rospub)
        asr.decode_asr()
        # rclpy.spin(asr.node)
    except Exception as exception:
        print("Error running decode_asr()"))
    
    asr.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()