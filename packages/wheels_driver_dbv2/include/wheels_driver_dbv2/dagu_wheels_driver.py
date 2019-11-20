#!/usr/bin/python

from Adafruit_MotorHAT import Adafruit_MotorHAT
import math
import rospy
import pigpio
from time import sleep


class DaguWheelsDriver:
    """Class handling communication with motors.

        Wraps the Adafruit API to talk to DC motors with a simpler interface.
        The class contains methods for creating PWM signals according to
        requested velocities. Also contains hardware addresses related to the
        motors.

        Args:
            debug (:obj:`bool`): If `True`, will print a debug message every time a PWM
               signal is sent.

    """

    DC_MOTOR_MIN_PWM = 60  # Minimum speed for right motor
    DC_MOTOR_MAX_PWM = 255  # Maximum speed for right motor
    SPEED_TOLERANCE = 1.e-2  # speed tolerance level
    old_pwm_dc = 0.0  # TODO This is probly bad coding... fix such that you can use old_pwm_dc in the cb function for the wheel publisher

    def __init__(self, verbose=False, debug=False, left_flip=False, dcmotor_flip=False):

        self.motorhat = Adafruit_MotorHAT(addr=0x60)
        self.DcMotor = self.motorhat.getMotor(1)  # change the name from "rightMotor" to "DcMotor", RFMH_2019_02_26
        self.verbose = verbose or debug
        self.debug = debug
        self.dcmotor_sgn = 1.0  # line 34-36: changed everywhere "right_sgn" to "dcmotor_sgn", RFMH_2019_02_26
        if dcmotor_flip:
            self.dcmotor_sgn = -1.0
        self.dcmotorSpeed = 0.0  # change "rightSpeed" to "dcmotorSpeed", RFMH_2019_02_26
        self.updatePWM()
        self.old_pwm_dc = 0.0

    def PWMvalue(self, v, minPWM, maxPWM):
        """Transforms the requested speed into an int8 number.

            Args:
                v (:obj:`float`): requested speed, should be between -1 and 1.
                minPWM (:obj:`int8`): minimum speed as int8
                maxPWM (:obj:`int8`): maximum speed as int8
        """
        pwm = 0
        if math.fabs(v) > self.SPEED_TOLERANCE:
            pwm = int(math.floor(math.fabs(v) * (maxPWM - minPWM) + minPWM))
        return min(pwm, maxPWM)

    def updatePWM(self):
        """Sends commands to the microcontroller.

            Updates the current PWM signals (left and right) according to the
            linear velocities of the motors. The requested speed gets
            tresholded.
        """
        v_dc = self.dcmotorSpeed * self.dcmotor_sgn  # changed "vr" to "v_dc", "rightSpeed" to "dcmotorSpeed" and "right_sgn" to dcmotor_sgn", RFMH_2019_02_26
        pwm_dc = self.PWMvalue(v_dc, self.DC_MOTOR_MIN_PWM,
                               self.DC_MOTOR_MAX_PWM)  # changed "pwmr" to "pwm_dc" and "vr" to "v_dc" and adjusted both orange constants to "DC_MOTOR_MIN_PWM" AND "DC_MOTOR_MAX_PWM", RFMH_2019_02_26

        # TODO: Fix this debug message. I am trying to port this code over from an old version, and I do not know
        #  what v and u are supposed to be here. Timothy Scott, 5.11.2019
        # if self.debug:  # where the duck does the "u" come from?!?, RFMH_2019_02_26
        #     print("v = %5.3f, u = %5.3f, v_dc = %5.3f, pwm_dc = %3d" % (
        #     v, u, v_dc, pwm_dc))  # deleted "vl" and "pwml" and adjust "vr" to "v_dc" to "pwm_dc"

        if math.fabs(v_dc) < self.SPEED_TOLERANCE:  # changed v_r to v_dc in if loop , RFMH_2019_02_28
            DcMotorMode = Adafruit_MotorHAT.RELEASE
            pwm_dc = 0
        elif v_dc > 0:
            DcMotorMode = Adafruit_MotorHAT.FORWARD
        elif v_dc < 0:
            DcMotorMode = Adafruit_MotorHAT.BACKWARD

        if not self.old_pwm_dc == pwm_dc:
            self.DcMotor.setSpeed(pwm_dc)  # changed rightMotor to DcMotor and pwmr to pwm_dc , RFMH_2019_02_28
            self.DcMotor.run(DcMotorMode)

        self.old_pwm_dc = pwm_dc

    def setWheelsSpeed(self, dc_motor_speed):
        """Sets speed of motors.

        Args:
           left (:obj:`float`): speed for the left wheel, should be between -1 and 1
           right (:obj:`float`): speed for the right wheel, should be between -1 and 1

        """
        self.dcmotorSpeed = dc_motor_speed  # changed rightSpeed to dcmotorSpeed and right to
        self.updatePWM()

    def __del__(self):
        """Destructor method.

            Releases the motors and deletes tho object.
        """
        self.DcMotor.run(Adafruit_MotorHAT.RELEASE)  # changed rightMotor to DcMotor , RFMH_2019_02_28
        del self.motorhat


class ServoDriver:
    # bus = smbus.SMBus(1)
    # DEVICE_ADDRESS = 0x60
    # DEVICE_REG = 0x3C
    SERVO_MAX_PWM = 98333.3                                                     # Maximum duty cycle for Servo
    SERVO_MIN_PWM = 51666.7                                                     # Minimum duty cycle for Servo
    # SERVO_MAX_I2C = 178
    # SERVO_MIN_I2C = 78

    def __init__(self):
        self.pi = pigpio.pi()  # Initialise Pi connection, RFMH_2019_03_21
        self.pi.set_mode(18, pigpio.OUTPUT)  # GPIO 12 as output, RFMH_2019_03_21
        self.pi.hardware_PWM(18, 50, 75000)                                     #initialize Servo with 50Hz and 75000 as middle of dutycycle , RFMH_2019_03_21
        #Write a single register
        #self.bus.write_byte_data(self.DEVICE_ADDRESS, self.DEVICE_REG, 128)              #


    def SetAngle(self, angle, servo_duty_limit, trim):                          #function to set the angle on the servo, RFMH_2019_03_21
        duty = (angle + servo_duty_limit - trim) * (self.SERVO_MAX_PWM - self.SERVO_MIN_PWM) / (2.0 * servo_duty_limit) + self.SERVO_MIN_PWM + trim * (50000.0 / 90) #duty is defined in library to go from 0 = 0% to 1'000'000 = 100%) , RFMH_2019_03_21
        self.pi.hardware_PWM(18, 50, duty)                                      #set hardware_PWM with the calculated duty
        rospy.loginfo("duty = %f" %duty)                                        #for debugging
        #write to i2c register
        # i2cduty = int((angle + servo_duty_limit - trim) * (self.SERVO_MAX_I2C - self.SERVO_MIN_I2C) / (2.0 * servo_duty_limit) + self.SERVO_MIN_I2C + trim * (255.0 / 180.0))
        # self.bus.write_byte_data(self.DEVICE_ADDRESS, self.DEVICE_REG, i2cduty)
        # rospy.loginfo("i2cduty = %i" %i2cduty)

    def __del__(self):
        self.pi.stop()                                                          #shutting down procedure for the gain_servo, RFMH_2019_03_21


# Simple example to test motors
if __name__ == '__main__':

    N = 10
    delay = 100. / 1000.

    dagu = DaguWheelsDriver()

    # turn left
    dagu.setSteerAngle(1.0)
    # accelerate forward
    for i in range(N):
        dagu.setSpeed((1.0 + i) / N)
        sleep(delay)
    # decelerate forward
    for i in range(N):
        dagu.setSpeed((-1.0 - i + N) / N)
        sleep(delay)

    # turn right
    dagu.setSteerAngle(-1.0)
    # accelerate backward
    for i in range(N):
        dagu.setSpeed(-(1.0 + i) / N)
        sleep(delay)
    # decelerate backward
    for i in range(N):
        dagu.setSpeed(-(-1.0 - i + N) / N)
        sleep(delay)

    # turn left
    dagu.setSteerAngle(1.0)
    # accelerate forward
    for i in range(N):
        dagu.setSpeed((1.0 + i) / N)
        sleep(delay)
    # decelerate forward
    for i in range(N):
        dagu.setSpeed((-1.0 - i + N) / N)
        sleep(delay)

    del dagu
