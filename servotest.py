from adafruit_servokit import ServoKit
import time
kit = ServoKit(channels=16)
try:
    while True:
        kit.servo[15].angle = 90
except KeyboardInterrupt:
    print("end")

