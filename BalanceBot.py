from mpu6050 import mpu6050
mpu = mpu6050(0x68)

try:
    while True:
        gyro_data = mpu.get_gyro_data()
        print("Gyro X : "+str(gyro_data['x']))
        print("Gyro Y : "+str(gyro_data['y']))
        print("Gyro Z : "+str(gyro_data['z']))
        print()
        print("-------------------------------")

except KeyboardInterrupt:
    print("Stopped")
