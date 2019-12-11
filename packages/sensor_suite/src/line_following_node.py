#!/usr/bin/env python
from duckietown import DTROS
from sensor_suite.line_following_sensor import LineFollower
import rospy


class LineFollowingNode(DTROS):

    def __init__(self, node_name='line_following_node'):
        super(LineFollowingNode, self).__init__(node_name)

        self.line_follower = LineFollower()

        _, valid = self.line_follower.read()
        if not valid:
            raise Exception('ADC readings for line following sensors are not all valid')

        self.timer = rospy.Timer(rospy.Duration.from_sec(1), self.publish_line_readings)

    def publish_line_readings(self, event):
        voltages, valid = self.line_follower.read()
        self.log(voltages)


if __name__ == "__main__":
    node = LineFollowingNode( )
    rospy.spin()
