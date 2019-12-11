#!/usr/bin/env python
from duckietown import DTROS
from sensor_suite.tof_driver import ToF
import rospy
import smbus

MUX_ADDR = 0x70
TOF_CHIP_ID = 0xAD02


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
                    self.tofs.append((i, tof))
                    tof.begin()
            except IOError:
                # IOError probably means that there is no ToF sensor plugged into this particular port of the mux
                pass

        if len(self.tofs) == 0:
            raise Exception('No valid ToF sensors found')

        self.timer = rospy.Timer(rospy.Duration.from_sec(1), self.read_distances)

    def select_tof(self, index):
        self.smbus.write_byte(MUX_ADDR, 1 << index)

    def read_distances(self, event):
        for index, tof in self.tofs:
            self.select_tof(index)
            distance, valid_pixels, confidence_value = tof.takeMeasurement()
            print("[{i}] Distance: {dist}, valid pixels: {pixels}, confidence: {conf}".format(
                i=index, dist=distance, pixels=valid_pixels, conf=confidence_value
            ))


if __name__ == "__main__":
    node = ToFNode()
    rospy.spin()