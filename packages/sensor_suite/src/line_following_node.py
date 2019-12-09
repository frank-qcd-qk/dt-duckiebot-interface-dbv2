#!/usr/bin/env python
from duckietown import DTROS
from sensor_suite.line_following_sensor import LineFollower
from duckietown_msgs.msg import LineFollowerStamped
from std_msgs.msg import Float32
import rospy


class LineFollowingNode(DTROS):

    def __init__(self, node_name='line_following_node'):
        super(LineFollowingNode, self).__init__(node_name)

        self.line_follower = LineFollower()

        _, valid = self.line_follower.read()
        if not valid:
            raise Exception('ADC readings for line following sensors are not all valid')

        self.pub = rospy.Publisher("~line_follower", LineFollowerStamped, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration.from_sec(0.1), self.publish_line_readings)

    def publish_line_readings(self, event):
        voltages, valid = self.line_follower.read()
        msg = LineFollowerStamped()
        msg.valid = valid
        msg.outer_right = voltages.outer_right / 3.3
        msg.inner_right = voltages.inner_right / 3.3
        msg.inner_left = voltages.inner_left / 3.3
        msg.outer_left = voltages.outer_left / 3.3

        self.pub.publish(msg)


if __name__ == "__main__":
    node = LineFollowingNode( )
    rospy.spin()
