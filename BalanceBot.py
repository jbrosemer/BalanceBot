import adafruit_mpu6050
import time
import adafruit_mpu6050
mpu = adafruit_mpu6050.MPU6050(0x68)
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)
try:
    while True:
        print("Gyro X:%.2f, Y: %.2f, Z: %.2f degrees/s" % (mpu.gyro))
        # if gyro_data['y'] > 100:
        #    kit.continuous_servo[0].throttle = 1
        # if gyro_data['y'] < -100:
        #    kit.continuous_servo[0].throttle = -1

except KeyboardInterrupt:
    print("Stopped")
