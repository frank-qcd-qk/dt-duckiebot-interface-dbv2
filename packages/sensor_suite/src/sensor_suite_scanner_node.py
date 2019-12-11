#!/usr/bin/env python
import smbus
import time
import sys
import os
import rospy
import datetime

from sensor_suite.msg import ScanningSensors
# define RFD77402_ADDR 0x4C //7-bit unshifted default I2C Address

class sensorScanning(object):

    def __init__(self, sensor_name):
        self.sensor_name = sensor_name

        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]

        self.MyBus=smbus.SMBus(1)

        #Addresses for Mux, Tof and the register of the ToF Chip ID
        self.muxaddress = 0x70
        self.tofaddr = 0x4C
        self.RFD77402_MOD_CHIP_ID = 0x28
        self.DEVICE_ID_TOF2       = 0xAD02
        self.DEVICE_ID_TOF1       = 0xAD02

        #Addresses from IMU and register address of IMU
        self.MPU9250_ADDRESS = 0x68
        self.MPU9250_CHIP_ID = 0x75
        self.DEVICE_ID_IMU   = 0x71

        #Address from line following sensor and register addres of ADC diagnostic
        self.LINEFOLLOWER_ADDRESS = 0x42
        self.ADC_DIAGNOSTIC = 0x8

        self.pub_scanning_sensors = rospy.Publisher("~scanning_sensors", ScanningSensors, queue_size=1)

    def scanningFrontBumper(self):
        try:
            self.MyBus.write_byte(0x70, 0x00)
            return True
        except:
            return False

    def scanningToF(self, mux_port):
        self.MyBus.write_byte(0x70, mux_port)
        time.sleep(0.001)
        try:
            mod_chip_ID = self.MyBus.read_word_data(self.tofaddr,self.RFD77402_MOD_CHIP_ID)
            print'Chip ID:', mod_chip_ID					# Chip ID must be 0xad01 or 0xad02
        except                                                                                                                                                                                                                                                                                          IOError as (errno, strerror):
                print "I/O error({0}): {1}".format(errno, strerror)
        try:
            if mod_chip_ID == self.DEVICE_ID_TOF1 or mod_chip_ID == self.DEVICE_ID_TOF2:
                return True
            else:
                return False
        except:
                return False

    def scanningIMU(self):
        try:
            chip_ID = self.MyBus.read_byte_data(self.MPU9250_ADDRESS, self.MPU9250_CHIP_ID)
        except IOError as (errno, strerror):
            print "I/O error({0}): {1}".format(errno, strerror)

        try:
            if chip_ID == self.DEVICE_ID_IMU:
                return True
            else:
                return False
        except:
            return False

    def scanningLF(self,adc_number):
        try:
            adcDiagnostic = self.MyBus.read_byte_data(self.LINEFOLLOWER_ADDRESS, self.ADC_DIAGNOSTIC)
        except IOError as (errno, strerror):
            print "I/O error({0}): {1}".format(errno, strerror)
        if (adcDiagnostic & adc_number) == 0:
            return True
        else:
            return False


    def msg_setting(self):
        self.msg_sensorscanning = ScanningSensors()
        self.msg_sensorscanning.state_front_bumper = self.scanningFrontBumper()
        print self.scanningFrontBumper()
        self.msg_sensorscanning.state_tof_fl = self.scanningToF(64)
        print self.scanningToF(64)
        self.msg_sensorscanning.state_tof_fm = self.scanningToF(32)
        print self.scanningToF(32)
        self.msg_sensorscanning.state_tof_fr = self.scanningToF(2)
        print self.scanningToF(2)
        self.msg_sensorscanning.state_tof_sl = self.scanningToF(128)
        print self.scanningToF(128)
        self.msg_sensorscanning.state_tof_sr = self.scanningToF(1)
        print self.scanningToF(1)
        self.msg_sensorscanning.state_tof_bl = self.scanningToF(16)
        print self.scanningToF(16)
        self.msg_sensorscanning.state_tof_bm = self.scanningToF(8)
        print self.scanningToF(8)
        self.msg_sensorscanning.state_tof_br = self.scanningToF(4)
        print self.scanningToF(4)
        self.msg_sensorscanning.state_imu = self.scanningIMU()
        print self.scanningIMU()
        self.msg_sensorscanning.state_lf_outer_right = self.scanningLF(0x03)
        print self.scanningLF(0x03)
        self.msg_sensorscanning.state_lf_inner_right = self.scanningLF(0x0c)
        print self.scanningLF(0x0c)
        self.msg_sensorscanning.state_lf_outer_left = self.scanningLF(0x30)
        print self.scanningLF(0x30)
        self.msg_sensorscanning.state_lf_inner_left = self.scanningLF(0xc0)
        print self.scanningLF(0xc0)


        self.pub_scanning_sensors.publish(self.msg_sensorscanning)

if __name__ == '__main__':
    rospy.init_node('~sensor_scanner', anonymous=False)
    sensor_scanner = sensorScanning('sensor_scanner')
    sensor_scanner.msg_setting()
