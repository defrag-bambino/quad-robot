import sys
import time

from board import SCL, SDA
import busio

from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
pca.frequency = 60

# read the servo id from the first argument
servo_id = int(sys.argv[1])

servo = servo.Servo(pca.channels[servo_id])
servo.fraction = 0.5


while True:
    # get user input
    angle = input("Enter fractional angle: ")
    # set the servo angle
    servo.fraction = float(angle)
    time.sleep(0.1)

pca.deinit()
