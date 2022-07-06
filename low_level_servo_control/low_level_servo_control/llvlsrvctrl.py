import rclpy
from rclpy.node import Node

from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

from utility.constants import LegJointsEnum
from msgs_srvs_acts.msg import ServoAngles

import time


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

            self.home_pos = [0. for i in range(17)]
            self.home_pos[LegJointsEnum.FL_COXA] = 0.3
            self.home_pos[LegJointsEnum.FL_FEMUR] = 1.0
            self.home_pos[LegJointsEnum.FL_TIBIA] = 0.75
            self.home_pos[LegJointsEnum.FR_COXA] = 0.7
            self.home_pos[LegJointsEnum.FR_FEMUR] = 0.0
            self.home_pos[LegJointsEnum.FR_TIBIA] = 0.25
            self.home_pos[LegJointsEnum.BL_COXA] = 0.7
            self.home_pos[LegJointsEnum.BL_FEMUR] = 0.0
            self.home_pos[LegJointsEnum.BL_TIBIA] = 0.25
            self.home_pos[LegJointsEnum.BR_COXA] = 0.3
            self.home_pos[LegJointsEnum.BR_FEMUR] = 1.0
            self.home_pos[LegJointsEnum.BR_TIBIA] = 0.75
            self.home_pos[LegJointsEnum.HEAD_YAW] = 0.5
            self.home_pos[LegJointsEnum.HEAD_PITCH] = 0.6
            self.set_servo_angles(self.home_pos)

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
            # set the servo angles to the corresponding part of the leg joints, if they are above or equal to zero.
            # Otherwise, keep the current desired servo angle.
            self.desired_servo_angles[LegJointsEnum.FL_COXA] = msg.angles[LegJointsEnum.FL_COXA] if msg.angles[LegJointsEnum.FL_COXA] >= 0.0 else self.desired_servo_angles[LegJointsEnum.FL_COXA]
            self.desired_servo_angles[LegJointsEnum.FL_FEMUR] = msg.angles[LegJointsEnum.FL_FEMUR] if msg.angles[LegJointsEnum.FL_FEMUR] >= 0.0 else self.desired_servo_angles[LegJointsEnum.FL_FEMUR]
            self.desired_servo_angles[LegJointsEnum.FL_TIBIA] = msg.angles[LegJointsEnum.FL_TIBIA] if msg.angles[LegJointsEnum.FL_TIBIA] >= 0.0 else self.desired_servo_angles[LegJointsEnum.FL_TIBIA]
            self.desired_servo_angles[LegJointsEnum.FR_COXA] = msg.angles[LegJointsEnum.FR_COXA] if msg.angles[LegJointsEnum.FR_COXA] >= 0.0 else self.desired_servo_angles[LegJointsEnum.FR_COXA]
            self.desired_servo_angles[LegJointsEnum.FR_FEMUR] = msg.angles[LegJointsEnum.FR_FEMUR] if msg.angles[LegJointsEnum.FR_FEMUR] >= 0.0 else self.desired_servo_angles[LegJointsEnum.FR_FEMUR]
            self.desired_servo_angles[LegJointsEnum.FR_TIBIA] = msg.angles[LegJointsEnum.FR_TIBIA] if msg.angles[LegJointsEnum.FR_TIBIA] >= 0.0 else self.desired_servo_angles[LegJointsEnum.FR_TIBIA]
            self.desired_servo_angles[LegJointsEnum.BL_COXA] = msg.angles[LegJointsEnum.BL_COXA] if msg.angles[LegJointsEnum.BL_COXA] >= 0.0 else self.desired_servo_angles[LegJointsEnum.BL_COXA]
            self.desired_servo_angles[LegJointsEnum.BL_FEMUR] = msg.angles[LegJointsEnum.BL_FEMUR] if msg.angles[LegJointsEnum.BL_FEMUR] >= 0.0 else self.desired_servo_angles[LegJointsEnum.BL_FEMUR]
            self.desired_servo_angles[LegJointsEnum.BL_TIBIA] = msg.angles[LegJointsEnum.BL_TIBIA] if msg.angles[LegJointsEnum.BL_TIBIA] >= 0.0 else self.desired_servo_angles[LegJointsEnum.BL_TIBIA]
            self.desired_servo_angles[LegJointsEnum.BR_COXA] = msg.angles[LegJointsEnum.BR_COXA] if msg.angles[LegJointsEnum.BR_COXA] >= 0.0 else self.desired_servo_angles[LegJointsEnum.BR_COXA]
            self.desired_servo_angles[LegJointsEnum.BR_FEMUR] = msg.angles[LegJointsEnum.BR_FEMUR] if msg.angles[LegJointsEnum.BR_FEMUR] >= 0.0 else self.desired_servo_angles[LegJointsEnum.BR_FEMUR]
            self.desired_servo_angles[LegJointsEnum.BR_TIBIA] = msg.angles[LegJointsEnum.BR_TIBIA] if msg.angles[LegJointsEnum.BR_TIBIA] >= 0.0 else self.desired_servo_angles[LegJointsEnum.BR_TIBIA]
            self.desired_servo_angles[LegJointsEnum.HEAD_YAW] = msg.angles[LegJointsEnum.HEAD_YAW] if msg.angles[LegJointsEnum.HEAD_YAW] >= 0.0 else self.desired_servo_angles[LegJointsEnum.HEAD_YAW]
            self.desired_servo_angles[LegJointsEnum.HEAD_PITCH] = msg.angles[LegJointsEnum.HEAD_PITCH] if msg.angles[LegJointsEnum.HEAD_PITCH] >= 0.0 else self.desired_servo_angles[LegJointsEnum.HEAD_PITCH]


        def set_servo_angles(self, servo_angles, sleep_between_ms=0.):
            # set the servo angles to given servo angles
            self.servos[LegJointsEnum.FL_COXA].fraction = servo_angles[LegJointsEnum.FL_COXA]
            if sleep_between_ms > 0.:
                time.sleep(sleep_between_ms / 1000.)
            self.servos[LegJointsEnum.FL_FEMUR].fraction = servo_angles[LegJointsEnum.FL_FEMUR]
            if sleep_between_ms > 0.:
                time.sleep(sleep_between_ms / 1000.)
            self.servos[LegJointsEnum.FL_TIBIA].fraction = servo_angles[LegJointsEnum.FL_TIBIA]
            if sleep_between_ms > 0.:
                time.sleep(sleep_between_ms / 1000.)
            self.servos[LegJointsEnum.FR_COXA].fraction = servo_angles[LegJointsEnum.FR_COXA]
            if sleep_between_ms > 0.:
                time.sleep(sleep_between_ms / 1000.)
            self.servos[LegJointsEnum.FR_FEMUR].fraction = servo_angles[LegJointsEnum.FR_FEMUR]
            if sleep_between_ms > 0.:
                time.sleep(sleep_between_ms / 1000.)
            self.servos[LegJointsEnum.FR_TIBIA].fraction = servo_angles[LegJointsEnum.FR_TIBIA]
            if sleep_between_ms > 0.:
                time.sleep(sleep_between_ms / 1000.)
            self.servos[LegJointsEnum.BL_COXA].fraction = servo_angles[LegJointsEnum.BL_COXA]
            if sleep_between_ms > 0.:
                time.sleep(sleep_between_ms / 1000.)
            self.servos[LegJointsEnum.BL_FEMUR].fraction = servo_angles[LegJointsEnum.BL_FEMUR]
            if sleep_between_ms > 0.:
                time.sleep(sleep_between_ms / 1000.)
            self.servos[LegJointsEnum.BL_TIBIA].fraction = servo_angles[LegJointsEnum.BL_TIBIA]
            if sleep_between_ms > 0.:
                time.sleep(sleep_between_ms / 1000.)
            self.servos[LegJointsEnum.BR_COXA].fraction = servo_angles[LegJointsEnum.BR_COXA]
            if sleep_between_ms > 0.:
                time.sleep(sleep_between_ms / 1000.)
            self.servos[LegJointsEnum.BR_FEMUR].fraction = servo_angles[LegJointsEnum.BR_FEMUR]
            if sleep_between_ms > 0.:
                time.sleep(sleep_between_ms / 1000.)
            self.servos[LegJointsEnum.BR_TIBIA].fraction = servo_angles[LegJointsEnum.BR_TIBIA]
            if sleep_between_ms > 0.:
                time.sleep(sleep_between_ms / 1000.)
            self.servos[LegJointsEnum.HEAD_YAW].fraction = servo_angles[LegJointsEnum.HEAD_YAW]
            if sleep_between_ms > 0.:
                time.sleep(sleep_between_ms / 1000.)
            self.servos[LegJointsEnum.HEAD_PITCH].fraction = servo_angles[LegJointsEnum.HEAD_PITCH]
            if sleep_between_ms > 0.:
                time.sleep(sleep_between_ms / 1000.)

def main(args=None):
    rclpy.init(args=args)
    llvsctrl = LowLevelServoController()
    rclpy.spin(llvsctrl)

    llvsctrl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
