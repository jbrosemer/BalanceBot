from adafruit_servokit import ServoKit
import time
kit = ServoKit(channels=16)
try:
    while True:
        kit.servo[15].angle = 180
        time.sleep(2)
        kit.servo[15].angle = 0
except KeyboardInterrupt:
    print("end")
    
