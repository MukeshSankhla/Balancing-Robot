#!/usr/bin/env python3
"""
balance_robot.py

Python script to control a self-balancing robot. 
It interfaces with:
1. An ESP32 via USB Serial, which streams IMU data (MPU-6050 pitch angle).
2. Two DFRobot M0601 motors (ID 0x01 Right, ID 0x02 Left) via an RS485 USB adapter.

The script runs a PID loop at the rate of the incoming IMU stream (~100Hz), 
calculates corrective wheel speeds, and sends velocity commands to the motors.

Safety checks:
- Shuts down motors if the robot tilts past a threshold (default 35 degrees).
- Shuts down motors if communication with the ESP32 is lost.
"""

import serial
import time
import sys
import argparse
import signal
import math

# --- CRC8 Maxim Calculation ---
def crc8_maxim(data):
    """CRC-8/MAXIM: polynomial x^8 + x^5 + x^4 + 1, reflected 0x8C."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if (crc & 0x01) else (crc >> 1)
    return crc

# --- Motor Control Frame Builders ---
def build_frame(motor_id, cmd_byte, data_bytes):
    """Build a standard 10-byte M0601 frame: [ID, CMD, 7 data bytes, CRC]."""
    f = [motor_id, cmd_byte] + list(data_bytes)
    f.append(crc8_maxim(f))
    return bytes(f)

def frame_velocity_mode(motor_id):
    """Set the motor to Velocity Loop mode (mode 0x02)."""
    # Note: Mode switch uses specific protocol format
    return bytes([motor_id, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02])

def frame_velocity(motor_id, rpm, accel=1):
    """Build velocity command frame. rpm: -330 to +330."""
    rpm = max(-330, min(330, int(rpm)))
    v = rpm.to_bytes(2, 'big', signed=True)
    return build_frame(motor_id, 0x64, [v[0], v[1], accel, 0x00, 0x00, 0x00, 0x00])

def frame_brake(motor_id):
    """Build electric brake command frame."""
    return build_frame(motor_id, 0x64, [0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00])

# --- PID Controller ---
class PIDController:
    def __init__(self, kp, ki, kd, target=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target = target
        self.integral = 0.0
        self.last_error = 0.0
        
    def compute(self, current_value, gyro_rate, dt, kp_nonlin=0.0):
        """
        Compute PID output.
        Using direct gyro rate for D term is cleaner and less noisy than numeric differentiation.
        """
        error = current_value - self.target
        self.integral += error * dt
        
        # Anti-windup: clamp the integral term to prevent runaway error build-up
        self.integral = max(-50.0, min(50.0, self.integral))
        
        # Calculate active Kp with non-linear scaling for larger errors
        kp_active = self.kp * (1.0 + kp_nonlin * abs(error))
        
        # Proportional + Integral + Derivative (using direct gyro angular velocity)
        p_term = kp_active * error
        i_term = self.ki * self.integral
        d_term = self.kd * gyro_rate
        
        output = p_term + i_term + d_term
        return output

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0

def smooth_stop(ser_motor, current_speed_r, current_speed_l):
    """
    Ramps down the motors from their current speed to 0 RPM smoothly,
    then applies the electric brakes to lock the wheels.
    """
    print(f"\n[*] Smooth stop: deceleration ramp starting (R: {current_speed_r:.1f} RPM, L: {current_speed_l:.1f} RPM)")
    
    # Ramping down over 10 steps (20ms per step -> 200ms total deceleration)
    steps = 10
    for i in range(1, steps + 1):
        ratio = 1.0 - (i / steps)
        target_r = int(current_speed_r * ratio)
        target_l = int(current_speed_l * ratio)
        try:
            ser_motor.write(frame_velocity(0x01, target_r))
            time.sleep(0.001)
            ser_motor.write(frame_velocity(0x02, target_l))
            time.sleep(0.02)
        except Exception:
            break
            
    print("[*] Deceleration complete. Applying electric brakes to lock wheels.")
    for _ in range(5):
        try:
            ser_motor.write(frame_brake(0x01))
            time.sleep(0.01)
            ser_motor.write(frame_brake(0x02))
            time.sleep(0.01)
        except Exception:
            pass

def resolve_port(port_name):
    """
    Resolves shorthand port names and prepends /dev/ on Linux if necessary.
    E.g. tty0 -> /dev/ttyACM0 (or /dev/ttyUSB0)
         tty1 -> /dev/ttyUSB0 (or /dev/ttyACM1)
    """
    if not port_name:
        return port_name
    if sys.platform.startswith('win'):
        return port_name
        
    import os
    if port_name == "tty0":
        if os.path.exists("/dev/ttyACM0"):
            return "/dev/ttyACM0"
        elif os.path.exists("/dev/ttyUSB0"):
            return "/dev/ttyUSB0"
        return "/dev/ttyACM0"
    elif port_name == "tty1":
        if os.path.exists("/dev/ttyUSB0"):
            return "/dev/ttyUSB0"
        elif os.path.exists("/dev/ttyACM1"):
            return "/dev/ttyACM1"
        elif os.path.exists("/dev/ttyUSB1"):
            return "/dev/ttyUSB1"
        return "/dev/ttyUSB0"
        
    if not port_name.startswith('/dev/'):
        if port_name.startswith('tty'):
            return "/dev/" + port_name
    return port_name

# --- Main Program ---
def main():
    parser = argparse.ArgumentParser(description="Self-Balancing Robot Controller")
    parser.add_argument("--imu", type=str, default=None, help="IMU (RP2040) serial port. Falls back to --esp.")
    parser.add_argument("--esp", type=str, default=None, help="ESP32 serial port (deprecated, use --imu instead)")
    parser.add_argument("--motor", type=str, default=None, help="Motor RS485 serial port")
    parser.add_argument("--baud-esp", type=int, default=115200, help="Baud rate for IMU (default: 115200)")
    parser.add_argument("--baud-motor", type=int, default=115200, help="Baud rate for RS485 (default: 115200)")
    parser.add_argument("--kp", type=float, default=1.2, help="PID Proportional gain (default: 1.2)")
    parser.add_argument("--ki", type=float, default=0.0, help="PID Integral gain (default: 0.0)")
    parser.add_argument("--kd", type=float, default=0.03, help="PID Derivative gain (default: 0.03)")
    parser.add_argument("--target", type=float, default=0.0, help="Target balancing angle in degrees (offset calibration)")
    parser.add_argument("--limit", type=int, default=50, help="Max wheel speed in RPM (default: 50, limit: 330)")
    parser.add_argument("--safety-angle", type=float, default=35.0, help="Tilt angle threshold for emergency shutdown")
    parser.add_argument("--right-sign", type=int, default=-1, choices=[1, -1], help="Direction sign for Right Motor (default: -1)")
    parser.add_argument("--left-sign", type=int, default=1, choices=[1, -1], help="Direction sign for Left Motor (default: 1)")
    parser.add_argument("--kp-nonlin", type=float, default=0.05, help="Non-linear Kp error scaling coefficient (default: 0.05)")
    parser.add_argument("--kp-vel", type=float, default=0.01, help="Velocity feedback proportional gain (default: 0.01)")
    parser.add_argument("--ki-vel", type=float, default=0.0005, help="Velocity feedback integral gain (default: 0.0005)")
    parser.add_argument("--show-raw", action="store_true", help="Print raw accelerometer and gyroscope values from MPU-6050")
    parser.add_argument("--slew-limit", type=float, default=150.0, help="Max wheel speed change rate in RPM/sec (default: 150.0)")
    parser.add_argument("--imu-orientation", type=str, default="flat", choices=["flat", "vertical"], help="MPU6050 IMU mounting orientation (default: flat)")
    parser.add_argument("--imu-pitch-sign", type=float, default=1.0, help="Direction multiplier for accelerometer pitch (1.0 or -1.0, default: 1.0)")
    parser.add_argument("--imu-gyro-sign", type=float, default=1.0, help="Direction multiplier for gyroscope pitch rate (1.0 or -1.0, default: 1.0)")
    parser.add_argument("--alpha", type=float, default=0.98, help="Complementary filter coefficient (default: 0.98)")
    args = parser.parse_args()

    # Determine dynamic defaults based on OS platform
    is_windows = sys.platform.startswith('win')
    
    imu_raw = args.imu if args.imu is not None else args.esp
    if imu_raw is None:
        imu_raw = "COM10" if is_windows else "/dev/ttyACM0"
        
    motor_raw = args.motor
    if motor_raw is None:
        motor_raw = "COM13" if is_windows else "/dev/ttyUSB0"
        
    imu_port = resolve_port(imu_raw)
    motor_port = resolve_port(motor_raw)

    print("=" * 60)
    print("           M0601 Self-Balancing Robot PID Controller")
    print("=" * 60)
    print(f"IMU Port:     {imu_port} (Baud: {args.baud_esp})")
    print(f"Motor Port:   {motor_port} (Baud: {args.baud_motor})")
    print(f"PID Params:   Kp={args.kp:.2f}, Ki={args.ki:.3f}, Kd={args.kd:.3f} | Kp-Nonlin={args.kp_nonlin:.3f}")
    print(f"Vel Loop:     Kp-Vel={args.kp_vel:.4f}, Ki-Vel={args.ki_vel:.5f}")
    print(f"Balance Target: {args.target:.2f} deg")
    print(f"Safety Angle: +/- {args.safety_angle:.1f} deg")
    print(f"Slew Limit:   {args.slew_limit:.1f} RPM/sec")
    print(f"Right Wheel:  ID 0x01, Direction Sign: {args.right_sign}")
    print(f"Left Wheel:   ID 0x02, Direction Sign: {args.left_sign}")
    print(f"IMU Align:    {args.imu_orientation} (Pitch Sign: {args.imu_pitch_sign}, Gyro Sign: {args.imu_gyro_sign})")
    print(f"Filter Alpha: {args.alpha:.3f}")
    print("=" * 60)

    # Initialize serial ports
    try:
        ser_esp = serial.Serial(imu_port, args.baud_esp, timeout=0.5)
        print(f"[✓] Connected to IMU on {imu_port}")
    except serial.SerialException as e:
        print(f"[✗] Failed to open IMU serial port: {e}")
        sys.exit(1)

    try:
        ser_motor = serial.Serial(motor_port, args.baud_motor, timeout=0.1)
        print(f"[✓] Connected to RS485 Motor Bus on {motor_port}")
    except serial.SerialException as e:
        print(f"[✗] Failed to open RS485 serial port: {e}")
        ser_esp.close()
        sys.exit(1)

    # Initialize PID controller
    pid = PIDController(args.kp, args.ki, args.kd, args.target)

    # Active running flag
    running = True

    # Signal handler for clean exit on Ctrl+C
    def signal_handler(sig, frame):
        nonlocal running
        print("\n[!] Ctrl+C detected. Shutting down balancing loop...")
        running = False

    signal.signal(signal.SIGINT, signal_handler)

    # Set both motors to velocity loop mode on start
    print("[*] Setting motors to Velocity Loop Mode...")
    for _ in range(5):
        ser_motor.write(frame_velocity_mode(0x01))
        time.sleep(0.01)
        ser_motor.write(frame_velocity_mode(0x02))
        time.sleep(0.01)

    # Clear IMU input buffer to avoid lag/stale readings
    ser_esp.reset_input_buffer()
    
    last_time = time.time()
    last_dashboard_time = 0
    loop_count = 0
    last_commanded_speed = 0.0
    right_speed = 0.0
    left_speed = 0.0
    estimated_position = 0.0
    
    # Filter state
    pitch_filtered = None

    try:
        while running:
            # Read telemetry from IMU
            line = ser_esp.readline()
            if not line:
                # If readline timed out (exceeded 0.5s), shut down motors immediately
                print("\n[CRITICAL] IMU Telemetry lost! Timing out...")
                break

            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            # Clamp dt to reasonable bounds in case of system lag
            dt = max(0.001, min(0.1, dt))

            # Decode line
            try:
                line_str = line.decode('utf-8', errors='ignore').strip()
                if not line_str:
                    continue
                
                # Parse CSV format: ax,ay,az,gx,gy,gz
                values = line_str.split(',')
                if len(values) != 6:
                    continue
                
                ax, ay, az, gx, gy, gz = map(float, values)
                
                # Calculate accelerometer-based pitch based on chosen orientation
                if args.imu_orientation == "flat":
                    pitch_acc = math.degrees(math.atan2(ax, math.sqrt(ay**2 + az**2)))
                elif args.imu_orientation == "vertical":
                    pitch_acc = math.degrees(math.atan2(-az, math.sqrt(ay**2 + ax**2)))
                else:
                    pitch_acc = math.degrees(math.atan2(ax, az))
                
                # Apply sign corrections
                pitch_acc *= args.imu_pitch_sign
                gyro_y = gy * args.imu_gyro_sign  # rotation rate around Y-axis (pitch rate)
                
                # Complementary filter logic
                if pitch_filtered is None:
                    pitch_filtered = pitch_acc
                else:
                    pitch_filtered = args.alpha * (pitch_filtered + gyro_y * dt) + (1.0 - args.alpha) * pitch_acc
                
                pitch = pitch_filtered
                    
            except (ValueError, IndexError):
                # Ignore malformed packets during serial start
                continue

            # --- Safety Check: Emergency Stop if tilted past safety angle ---
            if abs(pitch) > args.safety_angle:
                print(f"\n[⚠️] EMERGENCY STOP: Robot exceeded safety angle! Pitch: {pitch:.2f}°")
                break

            # --- Velocity feedback to prevent runaway/drift ---
            # We integrate the commanded speed to estimate wheel position.
            estimated_position += last_commanded_speed * dt
            # Clamp estimated position to avoid windup
            estimated_position = max(-500.0, min(500.0, estimated_position))
            
            # The velocity feedback adjusts the target angle to lean the robot
            # in the opposite direction of motion, bringing it to a stop.
            velocity_target_offset = (args.kp_vel * last_commanded_speed) + (args.ki_vel * estimated_position)
            # Clamp the target pitch adjustment to a safe range (e.g. +/- 3.0 degrees)
            velocity_target_offset = max(-3.0, min(3.0, velocity_target_offset))
            
            # Adjust the PID target angle dynamically
            pid.target = args.target - velocity_target_offset

            # --- PID Control Computation ---
            control_output = pid.compute(pitch, gyro_y, dt, args.kp_nonlin)

            # Limit speeds to prevent runaway
            target_rpm = max(-args.limit, min(args.limit, control_output))
            
            # --- Slew Rate Limiter to prevent sudden jumps ---
            max_change = args.slew_limit * dt
            if target_rpm > last_commanded_speed + max_change:
                target_rpm = last_commanded_speed + max_change
            elif target_rpm < last_commanded_speed - max_change:
                target_rpm = last_commanded_speed - max_change
            last_commanded_speed = target_rpm

            # Apply motor direction signs
            # Right motor is ID 0x01
            right_speed = args.right_sign * target_rpm
            # Left motor is ID 0x02
            left_speed = args.left_sign * target_rpm

            # Send velocity commands over RS485 bus
            try:
                ser_motor.write(frame_velocity(0x01, right_speed))
                # Add a tiny delay to prevent collision/buffer overload on half-duplex RS485
                time.sleep(0.001) 
                ser_motor.write(frame_velocity(0x02, left_speed))
            except serial.SerialException as e:
                print(f"\n[CRITICAL] RS485 communication failure: {e}")
                break

            # Print dashboard every ~0.1 seconds to not flood console
            loop_count += 1
            if current_time - last_dashboard_time >= 0.1:
                last_dashboard_time = current_time
                if args.show_raw:
                    acc_str = f"Acc: {ax:+5.3f}X {ay:+5.3f}Y {az:+5.3f}Z"
                    gyr_str = f"Gyr: {gx:+6.1f}X {gy:+6.1f}Y {gz:+6.1f}Z"
                    print(f"\r[Loop: {loop_count:6d}] Pitch: {pitch:+6.2f}° | {acc_str} | {gyr_str}", end="", flush=True)
                else:
                    print(f"\r[Loop: {loop_count:6d}] Pitch: {pitch:+6.2f}° | GyroY: {gyro_y:+6.1f}°/s | Target RPM: {target_rpm:+6.1f} | R: {right_speed:+5.1f} | L: {left_speed:+5.1f}", end="", flush=True)

    except Exception as e:
        print(f"\n[ERROR] Unexpected error in control loop: {e}")
        
    finally:
        # Ramps down speeds smoothly to 0 RPM and then applies electric brakes
        smooth_stop(ser_motor, right_speed, left_speed)

        # Close serial ports safely
        try:
            ser_esp.close()
            print("[✓] IMU Serial Port closed.")
        except Exception:
            pass

        try:
            ser_motor.close()
            print("[✓] RS485 Motor Serial Port closed.")
        except Exception:
            pass

        print("Done. Goodbye!")

if __name__ == "__main__":
    main()
