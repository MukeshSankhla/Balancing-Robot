import serial
import time

PORT = "/dev/ttyACM0"   # Change if your Pico uses another port
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

print("Connected to", PORT)
print("Waiting for MPU6050 data...\n")

try:
    while True:
        line = ser.readline().decode("utf-8", errors="ignore").strip()

        if not line:
            continue

        try:
            values = line.split(",")

            if len(values) == 6:
                ax, ay, az, gx, gy, gz = map(float, values)

                print("--------------------------------")
                print("Acceleration (g)")
                print(f"X : {ax:8.3f}")
                print(f"Y : {ay:8.3f}")
                print(f"Z : {az:8.3f}")

                print("\nGyroscope (deg/s)")
                print(f"X : {gx:8.3f}")
                print(f"Y : {gy:8.3f}")
                print(f"Z : {gz:8.3f}")

            else:
                print("Raw:", line)

        except ValueError:
            print("Invalid data:", line)

except KeyboardInterrupt:
    print("\nStopped")

finally:
    ser.close()
