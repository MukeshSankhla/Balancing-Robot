from machine import Pin, I2C
import time
import struct

# I2C1 (your working setup)
i2c = I2C(1, scl=Pin(3), sda=Pin(2), freq=400000)

MPU6050_ADDR = 0x68

PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B

# Wake MPU6050
i2c.writeto_mem(MPU6050_ADDR, PWR_MGMT_1, b'\x00')
time.sleep(0.1)

def read_mpu6050():
    data = i2c.readfrom_mem(MPU6050_ADDR, ACCEL_XOUT_H, 14)
    return struct.unpack(">hhhhhhh", data)

while True:
    ax, ay, az, temp, gx, gy, gz = read_mpu6050()

    # Convert units
    ax_g = ax / 16384.0
    ay_g = ay / 16384.0
    az_g = az / 16384.0

    gx_dps = gx / 131.0
    gy_dps = gy / 131.0
    gz_dps = gz / 131.0

    # Send CSV:
    # ax,ay,az,gx,gy,gz
    print("{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}".format(
        ax_g,
        ay_g,
        az_g,
        gx_dps,
        gy_dps,
        gz_dps
    ))

    time.sleep_ms(10)   # 100 Hz output

