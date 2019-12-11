#!/usr/bin/env python
import time

import rospy
from duckietown import DTROS
from imu_driver import mpu9250


class IMUHandler(DTROS):
    def __init__(self, node_name):
        super(IMUHandler, self).__init__(node_name=node_name)
        # self.mux_port = mux_port

        self.current_state = True

        self.timer = rospy.Timer(rospy.Duration.from_sec(0.1), self.publish_data)
        self.sensor = mpu9250()

    def publish_data(self, event):
        sensor = mpu9250()
        try:
            a = sensor.accel
            print('Accel: {:.3f} {:.3f} {:.3f} mg'.format(*a))
            g = sensor.gyro
            print('Gyro: {:.3f} {:.3f} {:.3f} dps'.format(*g))
            # m = imu.mag
            # print 'Magnet: {:.3f} {:.3f} {:.3f} mT'.format(*m)
            # m = imu.temp
            # print 'Temperature: {:.3f} C'.format(m)
            time.sleep(0.001)

        except IOError as (errno, strerror):
            print("I/O error({0}): {1}".format(errno, strerror))


if __name__ == '__main__':
    imu_sensor = IMUHandler('imu_node')
    rospy.spin()