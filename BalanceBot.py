import adafruit_mpu6050

mpu = adafruit_mpu6050(0x68)
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)
try:
    while True:
        gyro_data = mpu.get_gyro_data()
        print("Gyro X : "+str(gyro_data['x']))
        print("Gyro Y : "+str(gyro_data['y']))
        print("Gyro Z : "+str(gyro_data['z']))
        print()
        print("-------------------------------")
        if gyro_data['y'] > 100:
            kit.continuous_servo[0].throttle = 1
        if gyro_data['y'] < -100:
            kit.continuous_servo[0].throttle = -1

except KeyboardInterrupt:
    print("Stopped")
