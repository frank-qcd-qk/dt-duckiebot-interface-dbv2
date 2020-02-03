#!/usr/bin/env python
from duckietown import DTROS
from duckietown_msgs.msg import ToFStamped
from sensor_suite.tof_driver import ToF
from sensor_suite import SensorNotFound
import rospy
import smbus
import yaml
import os
from collections import namedtuple

MUX_ADDR = 0x70
TOF_CHIP_ID = 0xAD02


class SensorData(namedtuple('sensor_data', ['index', 'tof', 'pub'])):
    """
    Represents all of the necessary data for communicating with 1 individual ToF sensor.

    index: 0 - 7 inclusive. Index on the I2C multiplexer
    tof: The actual ToF object
    pub: rospy publisher
    """


class ToFNode(DTROS):
    """
    This node performs I2C communication with RFD77402 time-of-flight (ToF) distance sensors.

    This node will try to communicate with each of the 8 ports on the multiplexer on the front bumper.
    For each ToF sensor it finds, it will publish a ROS topic called 'tof_<n>', where <n> is the number of the port
    on the multiplexer.
    If it finds no ToF sensors, this node should cleanly exit. If an uncaught exception results in a stack
    trace being displayed, this is a bug, and should be reported.
    """

    def __init__(self, node_name='tof_node'):
        super(ToFNode, self).__init__(node_name)
        self.veh_name = rospy.get_namespace().strip("/")
        self.parameters['~polling_hz'] = None
        self.parameters['~m'] = None
        self.parameters['~b'] = None

        self.readParamFromFile()
        self.updateParameters()

        self.smbus = smbus.SMBus(1)

        self.tofs = []

        for i in range(0, 8):
            try:
                self.select_tof(i)
                tof = ToF()
                chip_id = self.smbus.read_word_data(tof.addr, tof.RFD77402_MOD_CHIP_ID)
                if chip_id == TOF_CHIP_ID:
                    pub = rospy.Publisher("~tof_{}".format(i), ToFStamped, queue_size=10)
                    self.tofs.append(SensorData(i, tof, pub))
                    tof.begin()
            except IOError:
                # IOError probably means that there is no ToF sensor plugged into this particular port of the mux
                pass

        if len(self.tofs) == 0:
            raise SensorNotFound('No valid ToF sensors found')

        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0 / self.parameters['~polling_hz']), self.read_distances)

    def readParamFromFile(self):
        """
        Reads the saved parameters from `/data/config/calibrations/sensor_suite/tof/DUCKIEBOTNAME.yaml` or
        uses the default values if the file doesn't exist. Adjsuts the ROS paramaters for the node
        with the new values.

        """
        # Check file existence
        fname = self.getFilePath(self.veh_name)
        # Use the default values from the config folder if a robot-specific file does not exist.
        if not os.path.isfile(fname):
            self.log("Kinematics calibration file %s does not exist! Using the default values." % fname, type='warn')
        else:
            with open(fname, 'r') as in_file:
                try:
                    yaml_dict = yaml.load(in_file)
                except yaml.YAMLError as exc:
                    self.log("YAML syntax error. File: %s fname. Exc: %s" %(fname, exc), type='fatal')
                    rospy.signal_shutdown()
                    return

            # Set parameters using value in yaml file
            if yaml_dict is None:
                # Empty yaml file
                return
            for param_name in ["m", "b"]:
                param_value = yaml_dict.get(param_name)
                if param_name is not None:
                    rospy.set_param("~"+param_name, param_value)
                else:
                    # Skip if not defined, use default value instead.
                    pass

    def getFilePath(self, name):
        """
        Returns the path to the robot-specific configuration file,
        i.e. `/data/config/calibrations/sensor_suite/tof/DUCKIEBOTNAME.yaml`.

        Args:
            name (:obj:`str`): the Duckiebot name

        Returns:
            :obj:`str`: the full path to the robot-specific calibration file

        """
        cali_file_folder = '/data/config/calibrations/sensor_suite/tof/'
        cali_file = cali_file_folder + name + ".yaml"
        return cali_file

    def select_tof(self, index):
        """
        Selects which port of the multiplexer is currently being used.
        :param index: Index of the port on the multiplexer (0 - 7, inclusive)
        :return: None
        :raises: IOError if it is unable to communicate with the front bumper multiplexer
        """
        self.smbus.write_byte(MUX_ADDR, 1 << index)

    def read_distances(self, event):
        """
        Reads all connected ToF sensors and publishes the results.

        This function should be called periodically by a ROSPy Timer.
        """
        for sensor in self.tofs:
            msg = ToFStamped()
            try:
                self.select_tof(sensor.index)
                error_code, distance, valid_pixels, confidence_value = sensor.tof.takeMeasurement()
                distance = self.parameters['~m'][sensor.index] * distance + self.parameters['~b'][sensor.index]
            except IOError as e:
                self.log("IOError when reading from ToF: {}".format(e))
                error_code, distance, valid_pixels, confidence_value = 0x03, 0, 0, 0
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
    # except IOError as e:
    #     print("Error when trying to communicate with front bumper. Is it plugged in?")
