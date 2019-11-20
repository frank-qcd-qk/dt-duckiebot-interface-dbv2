#!/usr/bin/env python
import rospy
from duckietown import DTROS
from duckietown_msgs.msg import WheelsCmdDBV2Stamped, BoolStamped
from wheels_driver_dbv2.dagu_wheels_driver import DaguWheelsDriver, ServoDriver

class WheelsDriverNode(DTROS):
    """Node handling the motor velocities communication.

        Subscribes to the requested wheels commands (linear velocities, i.e. velocity for the left and
        the right wheels) and to an emergency stop flag. When the emergency flag `~emergency_stop` is set to
        `False` it actuates the wheel driver with the velocities received from `~wheels_cmd_dbv2`. Publishes
        the execution of the commands to `~wheels_cmd_executed`.

        The emergency flag is `False` by default.

        Subscribers:
           ~wheels_cmd (:obj:`WheelsCmdDBV2Stamped`): The requested wheel command
           ~emergency_stop (:obj:`BoolStamped`): Emergency stop. Can stop the actual execution of the
               wheel commands by the motors if set to `True`. Set to `False` for nominal operations.
        Publishers:
           ~wheels_cmd_executed (:obj:`WheelsCmdDBV2Stamped`): Publishes the actual commands executed, i.e. when
               the emergency flag is `False` it publishes the requested command, and when it is `True`: zero
               values for both motors.

    """

    def __init__(self, node_name):

        print("RUNNING WHEELS DRIVER DBV2 ==============================================")

        # Initialize the DTROS parent class
        super(WheelsDriverNode, self).__init__(node_name=node_name)

        self.estop = False

        # Parameters for maximal turning radius
        self.use_rad_lim = self.setupParam("~use_rad_lim", False)
        self.wheel_distance = self.setupParam("~wheel_distance",
                                              0.092)  # wheel_distance default value changed to 0.092, RFMH_2019_02_26
        self.servo_duty_limit = self.setupParam("~servo_duty_limit",
                                                42)  # shift angle by its geometric left limit for PWM modulation, RFMH_2019_04_23

        # Setup publishers
        self.DCdriver = DaguWheelsDriver()  # rename driver to DCdriver
        self.Servodriver = ServoDriver()  # creaste new object Servodriver, RFMH_2019_03_21

        # Initialize the executed commands message
        self.msg_wheels_cmd = WheelsCmdDBV2Stamped()

        # Publisher for wheels command wih execution time
        self.pub_wheels_cmd = self.publisher("~wheels_cmd_executed", WheelsCmdDBV2Stamped, queue_size=1)

        # Subscribers
        self.sub_topic = self.subscriber("~wheels_cmd_dbv2", WheelsCmdDBV2Stamped, self.cbWheelsCmdDBV2, queue_size=1)
        self.sub_e_stop = self.subscriber("~emergency_stop", BoolStamped, self.cbEStop, queue_size=1)

        self.log("Initialized.")

    def cbWheelsCmdDBV2(self, msg):
        """Callback that sets wheels' speeds.

            Creates the wheels' speed message and publishes it. If the
            emergency stop flag is activated, publishes zero command.

            Args:
                msg (WheelsCmdDBV2Stamped): velocity command
        """

        if self.estop:
            self.DCdriver.setWheelsSpeed(
                dc_motor_speed=0.0)  # replace the two arguments "left" and "right" by only one called "dc_motor_speed" and rename driver to DCdriver
            self.Servodriver.SetAngle(angle=0.0, servo_duty_limit=self.servo_duty_limit,
                                      trim=msg.trim)  # set angle of servo to zero if stopped
            return

        self.DCdriver.setWheelsSpeed(
            dc_motor_speed=msg.vel_wheel)  # deleted argument for vel_left, changed "right" to "dc_motor_speed" and "vel_right" to "vel_wheel", RFMH_2019_02_26
        self.Servodriver.SetAngle(angle=msg.gamma, servo_duty_limit=self.servo_duty_limit,
                                  trim=msg.trim)  # set the steering angle on the servo, RFMH_2019_03_05
        # Put the wheel commands in a message and publish
        self.msg_wheels_cmd.header = msg.header
        # Record the time the command was given to the wheels_driver
        self.msg_wheels_cmd.header.stamp = rospy.get_rostime()
        self.msg_wheels_cmd.gamma = msg.gamma  # TODO: check whether gamma is published correctly with this implementation
        self.msg_wheels_cmd.vel_wheel = msg.vel_wheel  # "vel_right" changed to "vel_wheel", RFMH_2019_02_26
        self.msg_wheels_cmd.trim = msg.trim  # included "trim" to the message "WheelsCmdDBV2Stamped", RFMH_2019_04_01
        self.pub_wheels_cmd.publish(self.msg_wheels_cmd)

    def cbEStop(self, msg):
        """Callback that enables/disables emergency stop

            Args:
                msg (BoolStamped): emergency_stop flag
        """

        self.estop = msg.data
        if self.estop:
            rospy.log("Emergency Stop Activated")
        else:
            rospy.log("Emergency Stop Released")

    def onShutdown(self):
        """Shutdown procedure.

        Publishes a zero velocity command at shutdown."""

        self.DCdriver.setWheelsSpeed(dc_motor_speed=0.0)  # changed (left=0.0, right=0.0) to (dc_motor_speed=0.0)
        self.Servodriver.SetAngle(angle=0.0, servo_duty_limit=self.servo_duty_limit,
                                  trim=0.0)  # on shutdown place steering to 0 degrees
        rospy.loginfo("[%s] Shutting down." % (rospy.get_name()))

        super(WheelsDriverNode, self).onShutdown()

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name,param_name,value))
        return value


if __name__ == '__main__':
    # Initialize the node with rospy
    node = WheelsDriverNode(node_name='wheels_driver_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
