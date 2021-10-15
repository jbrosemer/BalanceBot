from adafruit_servokit import ServoKit
import time
kit = ServoKit(channels=16)
try:
    while True:
        kit.servo[15].angle = 20
        time.sleep(5)
        kit.servo[15].angle = 180
except KeyboardInterrupt:
    print("end")
    kit.servo[15].angle = 2
