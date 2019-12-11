#!/usr/bin/env python
from duckietown import DTROS
from duckietown_msgs.msg import ToFStamped
from sensor_suite.tof_driver import ToF
from sensor_suite import SensorNotFound
import rospy
import smbus
from collections import namedtuple

MUX_ADDR = 0x70
TOF_CHIP_ID = 0xAD02

sensor_data = namedtuple('sensor_data', ['index', 'tof', 'pub'])


class ToFNode(DTROS):

    def __init__(self, node_name='tof_node'):
        super(ToFNode, self).__init__(node_name)
        self.smbus = smbus.SMBus(1)

        self.tofs = []

        for i in range(0, 8):
            self.select_tof(i)
            try:
                tof = ToF()
                chip_id = self.smbus.read_word_data(tof.addr, tof.RFD77402_MOD_CHIP_ID)
                if chip_id == TOF_CHIP_ID:
                    pub = rospy.Publisher("~tof_{}".format(i), ToFStamped, queue_size=10)
                    self.tofs.append( sensor_data(i, tof, pub))
                    tof.begin()
            except IOError:
                # IOError probably means that there is no ToF sensor plugged into this particular port of the mux
                pass

        if len(self.tofs) == 0:
            raise SensorNotFound('No valid ToF sensors found')

        self.timer = rospy.Timer(rospy.Duration.from_sec(0.1), self.read_distances)

    def select_tof(self, index):
        self.smbus.write_byte(MUX_ADDR, 1 << index)

    def read_distances(self, event):
        for sensor in self.tofs:
            self.select_tof(sensor.index)
            msg = ToFStamped()
            error_code, distance, valid_pixels, confidence_value = sensor.tof.takeMeasurement()
            msg.error = error_code
            msg.distance = distance
            msg.confidence = confidence_value
            sensor.pub.publish(msg)
            # print("[{i}] Distance: {dist}, valid pixels: {pixels}, confidence: {conf}".format(
            #     i=sensor.index, dist=distance, pixels=valid_pixels, conf=confidence_value
            # ))


if __name__ == "__main__":
    try:
        node = ToFNode()
        rospy.spin()
    except SensorNotFound as e:
        print(e)
