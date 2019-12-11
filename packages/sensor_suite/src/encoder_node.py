#!/usr/bin/env python
import math

import pigpio
import rospy
from duckietown import DTROS
from duckietown_msgs.msg import EncoderStamped
from duckietown_msgs.srv import SetValueResponse, SetValue


# Encoder node
# Authors: Merlin Hosner, Rafael Froehlich

class EncoderNode(DTROS):
    def __init__(self, node_name):
        super(EncoderNode, self).__init__(node_name=node_name)

        self.parameters['~pin_encoder'] = None
        self.parameters['~radius'] = None
        self.parameters['~holes_per_round'] = None
        self.parameters['~holes_per_calc'] = None
        self.updateParameters()

        # Setup the Publishers & Messages
        self.pub_encoder_velocity = rospy.Publisher("~encoder_velocity", EncoderStamped, queue_size=1)
        self.msg_encoder_velocity = EncoderStamped()
        self.global_count = 0

        rospy.loginfo("[%s] Initialized.", self.node_name)
        self.printValues()

        self.pigpio = pigpio.pi()
        self.pigpio.set_mode(self.parameters['~pin_encoder'], pigpio.INPUT)
        self.pigpio.set_pull_up_down(self.parameters['~pin_encoder'], pigpio.PUD_UP)

        self.timer = rospy.Timer(rospy.Duration.from_sec(0.1), self.cbEncoderCmd)

    def printValues(self):
        rospy.loginfo("[%s] pin_encoder: %s radius: %s holes_per_round: %s holes_per_calc: %s" % (
            self.node_name, self.parameters['~pin_encoder'], self.parameters['~radius'],
            self.parameters['~holes_per_round'], self.parameters['~holes_per_calc']))

    def cbEncoderCmd(self, event):  # ,msg):
        rospy.loginfo("Waiting for falling edge on GPIO port 26")  # for debugging

        count = 0
        prev_timestamp = rospy.get_rostime()
        while count < self.parameters['~holes_per_calc']:
            # print("wait for FALLING")
            self.pigpio.wait_for_edge(self.parameters['~pin_encoder'],
                               pigpio.FALLING_EDGE)  # wait for the analog signal to drop down and rise up again
            # print("wait for RISING")
            self.pigpio.wait_for_edge(self.parameters['~pin_encoder'], pigpio.RISING_EDGE)
            count += 1
            self.global_count += 1
            # print("count:", count)
        current_timestamp = rospy.get_rostime()
        dt = (current_timestamp.secs + current_timestamp.nsecs / 1.e9) - (
                prev_timestamp.secs + prev_timestamp.nsecs / 1.e9)  # get the time passed since the last stamp
        # print("dt=", dt)
        if dt < (5 * 10 ** 9):  # check whether the bot is really moving with a minimal speed
            velocity = self.parameters['~radius'] * 2 * math.pi * (
                        self.parameters['~holes_per_calc'] / self.parameters['~holes_per_round']) / dt
            # print("velocity:", velocity)
            self.msg_encoder_velocity.vel_encoder = velocity

        else:
            rospy.loginfo("The bot has been standing for too long and it doesn't make sense to calculate the velocity")
            self.msg_encoder_velocity.vel_encoder = 0

        self.msg_encoder_velocity.header.stamp = rospy.Time.now()
        self.msg_encoder_velocity.count = self.global_count
        self.pub_encoder_velocity.publish(self.msg_encoder_velocity)

    def __del__(self):
        self.pigpio.stop()  # clean up GPIO on normal exit


if __name__ == '__main__':
    encoder_node = EncoderNode('encoder_node')
    rospy.spin()
