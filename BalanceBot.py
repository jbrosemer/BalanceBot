
import os
import sys
import time
import smbus
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)
from imusensor.MPU9250 import MPU9250

address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()
try:
    while True:
        # imu.readSensor()
        # imu.computeOrientation()
        kit.continuous_servo[15].throttle = -1
        #print ("Accel x: {0} ; Accel y : {1} ; Accel z : {2}".format(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2]))
        # print ("Gyro x: {0} ; Gyro y : {1} ; Gyro z : {2}".format(imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2]))
        # print ("Mag x: {0} ; Mag y : {1} ; Mag z : {2}".format(imu.MagVals[0], imu.MagVals[1], imu.MagVals[2]))
        # print ("roll: {0} ; pitch : {1} ; yaw : {2}".format(imu.roll, imu.pitch, imu.yaw))
        # if imu.AccelVals[1] < -5:
            # kit.continuous_servo[0].throttle = -1
        # if imu.AccelVals[1] > 5:
            # kit.continuous_servo[0].throttle = 1
except KeyboardInterrupt:
    print()
    print("ended")