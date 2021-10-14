import adafruit_mpu6050
import time
import board
import adafruit_mpu6050
i2c = board.I2C()  # uses board.SCL and board.SDA
mpu = adafruit_mpu6050.MPU6050(i2c)
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
