#!/usr/bin/env python
import time

import rospy
from duckietown import DTROS
from duckietown_msgs.msg import Imu
from sensor_suite.imu_driver import mpu9250
from sensor_suite import SensorNotFound


class IMUHandler(DTROS):
    def __init__(self, node_name):
        super(IMUHandler, self).__init__(node_name=node_name)
        # self.mux_port = mux_port

        self.current_state = True
        try:
            self.sensor = mpu9250(1)
            _ = self.sensor.accel
            _ = self.sensor.gyro
        except IOError as e:
            raise SensorNotFound("IMU sensor not detected")

        self.pub = rospy.Publisher('~imu', Imu, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration.from_sec(0.1), self.publish_data)

    def publish_data(self, event):
        try:

            a = self.sensor.accel
            # print('Accel: {:.3f} {:.3f} {:.3f} mg'.format(*a))
            g = self.sensor.gyro
            # print('Gyro: {:.3f} {:.3f} {:.3f} dps'.format(*g))
            # m = imu.mag
            # print 'Magnet: {:.3f} {:.3f} {:.3f} mT'.format(*m)
            # m = imu.temp
            # print 'Temperature: {:.3f} C'.format(m)
            msg = Imu()
            msg.angular_velocity.x = g[0]
            msg.angular_velocity.y = g[1]
            msg.angular_velocity.z = g[2]
            msg.linear_acceleration.x = a[0]
            msg.linear_acceleration.y = a[1]
            msg.linear_acceleration.z = a[2]
            self.pub.publish(msg)

        except IOError as (errno, strerror):
            print("I/O error({0}): {1}".format(errno, strerror))


if __name__ == '__main__':
    try:
        imu_sensor = IMUHandler('imu_node')
        rospy.spin()
    except SensorNotFound as e:
        print(e)
