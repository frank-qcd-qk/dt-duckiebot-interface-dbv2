#!/usr/bin/env python
import math

import pigpio
import rospy
from duckietown import DTROS
from duckietown_msgs.msg import EncoderStamped
import threading
import signal


# Encoder node
# Authors: Merlin Hosner, Rafael Froehlich

class EncoderNode(DTROS):
    def __init__(self, node_name):
        super(EncoderNode, self).__init__(node_name=node_name)

        self.parameters['~pin_encoder'] = None
        self.parameters['~radius'] = None
        self.parameters['~holes_per_round'] = None
        self.parameters['~polling_hz'] = None
        self.updateParameters()

        # Setup the Publishers & Messages
        self.pub_encoder_velocity = rospy.Publisher("~encoder_velocity", EncoderStamped, queue_size=1)
        self.global_count = 0
        self.last_published_time = rospy.get_time()
        self.last_published_count = 0
        self.count_lock = threading.Lock()
        self._run = True
        self.current_level = 2
        self.last_tick = 0

        rospy.loginfo("[%s] Initialized.", self.node_name)

        self.pigpio = pigpio.pi()
        self.pigpio.set_mode(self.parameters['~pin_encoder'], pigpio.INPUT)
        self.pigpio.set_pull_up_down(self.parameters['~pin_encoder'], pigpio.PUD_UP)
        self.pigpio.callback(self.parameters['~pin_encoder'], pigpio.EITHER_EDGE, self.edge_callback)

        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0 / self.parameters['~polling_hz']), self.publish_msg)

    # def printValues(self):
    #     rospy.loginfo("[%s] pin_encoder: %s radius: %s holes_per_round: %s holes_per_calc: %s" % (
    #         self.node_name, self.parameters['~pin_encoder'], self.parameters['~radius'],
    #         self.parameters['~holes_per_round'], self.parameters['~holes_per_calc']))
    #
    # def cbEncoderCmd(self, event):  # ,msg):
    #     rospy.loginfo("Waiting for falling edge on GPIO port 26")  # for debugging
    #
    #     count = 0
    #     prev_timestamp = rospy.get_rostime()
    #     while count < self.parameters['~holes_per_calc']:
    #         # print("wait for FALLING")
    #         self.pigpio.wait_for_edge(self.parameters['~pin_encoder'],
    #                            pigpio.FALLING_EDGE)  # wait for the analog signal to drop down and rise up again
    #         # print("wait for RISING")
    #         self.pigpio.wait_for_edge(self.parameters['~pin_encoder'], pigpio.RISING_EDGE)
    #         count += 1
    #         self.global_count += 1
    #         # print("count:", count)
    #     current_timestamp = rospy.get_rostime()
    #     dt = (current_timestamp.secs + current_timestamp.nsecs / 1.e9) - (
    #             prev_timestamp.secs + prev_timestamp.nsecs / 1.e9)  # get the time passed since the last stamp
    #     # print("dt=", dt)
    #     if dt < (5 * 10 ** 9):  # check whether the bot is really moving with a minimal speed
    #         velocity = self.parameters['~radius'] * 2 * math.pi * (
    #                     self.parameters['~holes_per_calc'] / self.parameters['~holes_per_round']) / dt
    #         # print("velocity:", velocity)
    #         self.msg_encoder_velocity.vel_encoder = velocity
    #
    #     else:
    #         rospy.loginfo("The bot has been standing for too long and it doesn't make sense to calculate the velocity")
    #         self.msg_encoder_velocity.vel_encoder = 0
    #
    #     self.msg_encoder_velocity.header.stamp = rospy.Time.now()
    #     self.msg_encoder_velocity.count = self.global_count
    #     self.pub_encoder_velocity.publish(self.msg_encoder_velocity)

    def publish_msg(self, _):
        with self.count_lock:
            total_count = self.global_count
        now = rospy.get_time()
        diff = total_count - self.last_published_count
        dt = now - self.last_published_time
        holes_velocity = diff / dt  # Velocity in holes per second
        velocity = (holes_velocity / self.parameters['~holes_per_round']) * 2 * math.pi * self.parameters['~radius']

        msg = EncoderStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_encoder = velocity
        msg.count = total_count
        self.pub_encoder_velocity.publish(msg)

        self.last_published_time = now
        self.last_published_count = total_count

    def edge_callback(self, gpio_pin, level, tick):

        if tick - self.last_tick > 1000:
            print("Tick: {}, Last: {}, Diff: {}, Level: {}".format(tick, self.last_tick, tick - self.last_tick, level))
            self.last_tick = tick
            if level == 0:
                with self.count_lock:
                    self.global_count += 1
                print("Got a tick")

    def __del__(self):
        self.pigpio.stop()  # clean up GPIO on normal exit


if __name__ == '__main__':
    encoder_node = EncoderNode('encoder_node')
    rospy.spin()
