import rclpy
from rclpy.node import Node

from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

from utility.constants import LegJointsEnum
from msgs_srvs_acts import ServoAngles


class LowLevelServoController(Node):
        def __init__(self):
            super().__init__('low_level_servo_controller')

            # init the i2c bus and the PCA9685
            self.i2c = busio.I2C(SCL, SDA)
            self.pca = PCA9685(self.i2c)
            self.pca.frequency = 60

            # set up the servos
            self.servos = [servo.Servo(self.pca.channels[i]) for i in range(17)]
            self.desired_servo_angles = [0. for i in range(17)]

            # create the subscriber
            self.subscription = self.create_subscription(ServoAngles, 'desired_servo_angles', self.servo_angles_callback, 1)

            # create the publisher and its timer
            self.publisher = self.create_publisher(ServoAngles, 'current_servo_angles', 1)
            self.timer = self.create_timer(0.05, self.publish_servo_angles)

        def publish_servo_angles(self):
            # create the message
            msg = ServoAngles()
            # set the message angles to the current servo angles
            msg.angles[LegJointsEnum.FL_COXA] = self.servos[LegJointsEnum.FL_COXA].fraction
            msg.angles[LegJointsEnum.FL_FEMUR] = self.servos[LegJointsEnum.FL_FEMUR].fraction
            msg.angles[LegJointsEnum.FL_TIBIA] = self.servos[LegJointsEnum.FL_TIBIA].fraction
            msg.angles[LegJointsEnum.FR_COXA] = self.servos[LegJointsEnum.FR_COXA].fraction
            msg.angles[LegJointsEnum.FR_FEMUR] = self.servos[LegJointsEnum.FR_FEMUR].fraction
            msg.angles[LegJointsEnum.FR_TIBIA] = self.servos[LegJointsEnum.FR_TIBIA].fraction
            msg.angles[LegJointsEnum.BL_COXA] = self.servos[LegJointsEnum.BL_COXA].fraction
            msg.angles[LegJointsEnum.BL_FEMUR] = self.servos[LegJointsEnum.BL_FEMUR].fraction
            msg.angles[LegJointsEnum.BL_TIBIA] = self.servos[LegJointsEnum.BL_TIBIA].fraction
            msg.angles[LegJointsEnum.BR_COXA] = self.servos[LegJointsEnum.BR_COXA].fraction
            msg.angles[LegJointsEnum.BR_FEMUR] = self.servos[LegJointsEnum.BR_FEMUR].fraction
            msg.angles[LegJointsEnum.BR_TIBIA] = self.servos[LegJointsEnum.BR_TIBIA].fraction
            msg.angles[LegJointsEnum.HEAD_YAW] = self.servos[LegJointsEnum.HEAD_YAW].fraction
            msg.angles[LegJointsEnum.HEAD_PITCH] = self.servos[LegJointsEnum.HEAD_PITCH].fraction
            # publish the message
            self.publisher.publish(msg)


        def servo_angles_callback(self, msg):
            # set the servo angles to the corresponding part of the leg joints
            self.desired_servo_angles[LegJointsEnum.FL_COXA] = msg.angles[LegJointsEnum.FL_COXA]
            self.desired_servo_angles[LegJointsEnum.FL_FEMUR] = msg.angles[LegJointsEnum.FL_FEMUR]
            self.desired_servo_angles[LegJointsEnum.FL_TIBIA] = msg.angles[LegJointsEnum.FL_TIBIA]
            self.desired_servo_angles[LegJointsEnum.FR_COXA] = msg.angles[LegJointsEnum.FR_COXA]
            self.desired_servo_angles[LegJointsEnum.FR_FEMUR] = msg.angles[LegJointsEnum.FR_FEMUR]
            self.desired_servo_angles[LegJointsEnum.FR_TIBIA] = msg.angles[LegJointsEnum.FR_TIBIA]
            self.desired_servo_angles[LegJointsEnum.BL_COXA] = msg.angles[LegJointsEnum.BL_COXA]
            self.desired_servo_angles[LegJointsEnum.BL_FEMUR] = msg.angles[LegJointsEnum.BL_FEMUR]
            self.desired_servo_angles[LegJointsEnum.BL_TIBIA] = msg.angles[LegJointsEnum.BL_TIBIA]
            self.desired_servo_angles[LegJointsEnum.BR_COXA] = msg.angles[LegJointsEnum.BR_COXA]
            self.desired_servo_angles[LegJointsEnum.BR_FEMUR] = msg.angles[LegJointsEnum.BR_FEMUR]
            self.desired_servo_angles[LegJointsEnum.BR_TIBIA] = msg.angles[LegJointsEnum.BR_TIBIA]
            self.desired_servo_angles[LegJointsEnum.HEAD_YAW] = msg.angles[LegJointsEnum.HEAD_YAW]
            self.desired_servo_angles[LegJointsEnum.HEAD_PITCH] = msg.angles[LegJointsEnum.HEAD_PITCH]
            


            