import rclpy
from rclpy.node import Node

from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

from utility.constants import LegJointsEnum, HOME_POS, SERVO_MOVE_TIME_MS
import utility.easing_p as easing_p
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
            self.servos = [servo.Servo(self.pca.channels[i]) for i in range(16)]
            self.desired_servo_angles = [0. for i in range(16)]
            self.des_serv_ang_timestamp = [self.get_clock().now() for i in range(16)]

            # create the subscriber
            self.subscription = self.create_subscription(ServoAngles, 'desired_servo_angles', self.servo_angles_callback, 1)

            # create the publisher and its timer
            self.publisher = self.create_publisher(ServoAngles, 'current_servo_angles', 1)
            self.timer = self.create_timer(0.05, self.update_and_publish_servo_angles)

            # init the servo angles to the home position
            for leg_joint in LegJointsEnum():
                self.servos[leg_joint].angle = HOME_POS[leg_joint]
            self.set_servo_angles(self.home_pos, sleep_between_ms=150)

        def update_and_publish_servo_angles(self):
            now_time = self.get_clock().now()
            servo_move_duration = rclpy.Duration(SERVO_MOVE_TIME_MS / 1000.)
            for leg_joint in LegJointsEnum():
                time_since_des_serv_ang_timestamp = now_time - self.des_serv_ang_timestamp[leg_joint]
                fractional_move_duration = time_since_des_serv_ang_timestamp / servo_move_duration
                if fractional_move_duration <= 1.0:
                    self.servos[leg_joint].fraction = easing_p.CubicEaseInOut(fractional_move_duration) * (self.desired_servo_angles[leg_joint] - self.servos[leg_joint].angle) + self.servos[leg_joint].angle
                else:
                    self.servos[leg_joint].fraction = self.desired_servo_angles[leg_joint]
            self.set_servo_angles(self.servos[leg_joint].angle)
                
            # create the message
            msg = ServoAngles()
            # set the message angles to the current servo angles
            for leg_joint in LegJointsEnum():
                msg.angles[leg_joint] = self.servos[leg_joint].fraction
            # publish the message
            self.publisher.publish(msg)

        def servo_angles_callback(self, msg):
            # set the servo angles to the corresponding part of the leg joints, if they are above or equal to zero.
            # Otherwise, keep the current desired servo angle.
            # Also, set the timestamp of the desired servo angle to the current time. This is later used to smoothly
            # move the servos to the desired angles.
            for leg_joint in LegJointsEnum():
                if msg.angles[leg_joint] >= 0.0:
                    self.desired_servo_angles[leg_joint] = msg.angles[leg_joint]
                    self.des_serv_ang_timestamp[leg_joint] = self.get_clock().now()


        def set_servo_angles(self, servo_angles, sleep_between_ms=0.):
            # set the servo angles to given servo angles
            for leg_joint in LegJointsEnum():
                self.servos[leg_joint].angle = servo_angles[leg_joint]
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
