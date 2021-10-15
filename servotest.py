from adafruit_servokit import ServoKit
import time
kit = ServoKit(channels=16)
try:
    while True:
        kit.servo[15].angle = 20
except KeyboardInterrupt:
    print("end")
    kit.servo[15].angle = 2
