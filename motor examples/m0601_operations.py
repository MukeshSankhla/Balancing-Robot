"""
m0601_operations.py — Interactive motor control in all three modes.

Controls:
  F / B  — Forward / Backward at SPIN_RPM (velocity mode)
  1..5   — Velocity presets: 50, 100, 150, 200, 250 RPM
  S      — Stop (velocity = 0)
  K      — Brake (electric brake)
  V      — Switch to Velocity Loop mode
  C      — Switch to Current Loop mode
  P      — Switch to Position Loop mode
  I      — Query feedback (print once)
  Q      — Quit

Usage:
    pip install pyserial
    python m0601_operations.py
"""

import serial
import time
import sys
import os
import threading

COM_PORT   = "COM13"
BAUD_RATE  = 115200
MOTOR_ID   = 0x01
SPIN_RPM   = 100        # Default speed for F/B keys
POLL_HZ    = 50         # Polling frequency in Hz


def crc8_maxim(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if (crc & 0x01) else (crc >> 1)
    return crc


def build(motor_id, b1, data7):
    """Build a standard 10-byte frame: [ID, B1, 7 data bytes, CRC]."""
    f = [motor_id, b1] + list(data7)
    f.append(crc8_maxim(f))
    return bytes(f)


# ── Verified frames ───────────────────────────────────────────────────────────

def frame_velocity_mode(motor_id):
    """Switch to velocity loop (mode 0x02). No CRC on last byte — use exact bytes."""
    return bytes([motor_id, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02])

def frame_current_mode(motor_id):
    """Switch to current loop (mode 0x01)."""
    return bytes([motor_id, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01])

def frame_position_mode(motor_id):
    """Switch to position loop (mode 0x03). Motor must be <10 RPM first."""
    return bytes([motor_id, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03])

def frame_velocity(motor_id, rpm, accel=1):
    """Velocity command. rpm: -330 to +330."""
    rpm  = max(-330, min(330, rpm))
    v    = rpm.to_bytes(2, 'big', signed=True)
    return build(motor_id, 0x64, [v[0], v[1], accel, 0x00, 0x00, 0x00, 0x00])

def frame_current(motor_id, value):
    """Current command. value: -32767 to +32767 (= -8A to +8A)."""
    value = max(-32767, min(32767, value))
    v     = value.to_bytes(2, 'big', signed=True)
    return build(motor_id, 0x64, [v[0], v[1], 0x00, 0x00, 0x00, 0x00, 0x00])

def frame_position(motor_id, pos):
    """Position command. pos: 0 to 32767 (= 0° to 360°)."""
    pos = max(0, min(32767, pos))
    v   = pos.to_bytes(2, 'big', signed=False)
    return build(motor_id, 0x64, [v[0], v[1], 0x00, 0x00, 0x00, 0x00, 0x00])

def frame_brake(motor_id):
    """Electric brake. Valid in velocity mode only."""
    return build(motor_id, 0x64, [0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00])

def frame_feedback(motor_id):
    """Request full feedback (speed, current, position, temp, error)."""
    return build(motor_id, 0x74, [0x00] * 7)

def frame_id_query():
    return bytes([0xC8, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE])


def decode_feedback(data):
    """Parse a 10-byte Protocol-2 feedback frame into a readable dict."""
    if len(data) < 10:
        return None
    mode_map = {0x01: "Current", 0x02: "Velocity", 0x03: "Position"}
    raw_cur  = int.from_bytes(data[2:4], 'big', signed=True)
    raw_vel  = int.from_bytes(data[4:6], 'big', signed=True)
    temp_c   = data[6]
    u8_pos   = data[7]
    err      = data[8]
    return {
        "id":       data[0],
        "mode":     mode_map.get(data[1], f"0x{data[1]:02X}"),
        "current_a": round(raw_cur * 8.0 / 32767, 3),
        "speed_rpm": raw_vel,
        "temp_c":    temp_c,
        "position_deg": round(u8_pos * 360.0 / 255, 1),
        "error":    f"0x{err:02X}" + (" (OK)" if err == 0 else " ⚠ FAULT"),
    }


def getch():
    """Read a single keypress without pressing Enter (cross-platform)."""
    if os.name == 'nt':
        import msvcrt
        return msvcrt.getch().decode('utf-8', errors='ignore').upper()
    else:
        import tty, termios
        fd  = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1).upper()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    print("=" * 54)
    print("  M0601 Motor Operations — Interactive Control")
    print(f"  Port: {COM_PORT}  ID: 0x{MOTOR_ID:02X}  Default: {SPIN_RPM} RPM")
    print("=" * 54)

    try:
        ser = serial.Serial(
            port=COM_PORT, baudrate=BAUD_RATE,
            bytesize=8, parity='N', stopbits=1, timeout=0.3
        )
        print(f"[✓] Opened {COM_PORT}")
    except serial.SerialException as e:
        print(f"[✗] {e}")
        sys.exit(1)

    # Confirm motor is online
    print("[*] Checking for motor...")
    ser.reset_input_buffer()
    ser.write(frame_id_query())
    time.sleep(0.3)
    resp = ser.read_all()
    if resp:
        print(f"[✓] Motor responded: {resp.hex(' ').upper()}")
    else:
        print("[!] No motor response — check wiring and 18V power.")

    # Switch to velocity mode and initialise polling state
    for _ in range(5):
        ser.write(frame_velocity_mode(MOTOR_ID))
        time.sleep(0.02)
    print("[✓] Velocity loop mode set.\n")

    # Shared state between main thread (keyboard) and poll thread (serial writes)
    state = {
        "frame":   frame_velocity(MOTOR_ID, 0),   # Current frame to poll
        "polling": True,                            # Polling active flag
        "running": True,                            # Application running flag
        "mode":    "velocity",
    }
    lock = threading.Lock()

    def poll_loop():
        """Background thread: sends state['frame'] at POLL_HZ."""
        interval = 1.0 / POLL_HZ
        while state["running"]:
            with lock:
                if state["polling"]:
                    try:
                        ser.write(state["frame"])
                    except Exception:
                        pass
            time.sleep(interval)

    poll_thread = threading.Thread(target=poll_loop, daemon=True)
    poll_thread.start()

    print("Controls:")
    print("  F/B     Forward / Backward")
    print("  1-5     Speed presets (50/100/150/200/250 RPM)")
    print("  S       Stop   K  Brake")
    print("  V/C/P   Switch mode: Velocity / Current / Position")
    print("  I       Print live feedback")
    print("  Q       Quit\n")

    try:
        while True:
            print("Key: ", end="", flush=True)
            key = getch()
            print(key)

            with lock:
                if key == 'F':
                    state["frame"]   = frame_velocity(MOTOR_ID, SPIN_RPM)
                    state["polling"] = True
                    print(f"  ► Forward {SPIN_RPM} RPM")

                elif key == 'B':
                    state["frame"]   = frame_velocity(MOTOR_ID, -SPIN_RPM)
                    state["polling"] = True
                    print(f"  ◄ Backward {SPIN_RPM} RPM")

                elif key in ('1', '2', '3', '4', '5'):
                    rpm = int(key) * 50
                    state["frame"]   = frame_velocity(MOTOR_ID, rpm)
                    state["polling"] = True
                    print(f"  ► {rpm} RPM")

                elif key == 'S':
                    state["frame"]   = frame_velocity(MOTOR_ID, 0)
                    state["polling"] = True
                    print("  ■ Stopped (velocity = 0)")

                elif key == 'K':
                    state["frame"]   = frame_brake(MOTOR_ID)
                    state["polling"] = True
                    print("  ■ Brake applied")

                elif key == 'V':
                    state["mode"] = "velocity"
                    state["polling"] = False
                for _ in range(5):
                    ser.write(frame_velocity_mode(MOTOR_ID))
                    time.sleep(0.02)
                print("  ✓ Velocity Loop mode")

                elif key == 'C':
                    state["mode"] = "current"
                    state["polling"] = False
                    for _ in range(5):
                        ser.write(frame_current_mode(MOTOR_ID))
                        time.sleep(0.02)
                    state["frame"]   = frame_current(MOTOR_ID, 0)
                    state["polling"] = True
                    print("  ✓ Current Loop mode — sending 0A")

                elif key == 'P':
                    state["mode"] = "position"
                    state["polling"] = False
                    for _ in range(5):
                        ser.write(frame_position_mode(MOTOR_ID))
                        time.sleep(0.02)
                    pos = int(input("  Enter position (0–32767): "))
                    state["frame"]   = frame_position(MOTOR_ID, pos)
                    state["polling"] = True
                    print(f"  ✓ Position Loop — targeting {pos} ({pos*360/32767:.1f}°)")

                elif key == 'I':
                    state["polling"] = False
                    time.sleep(0.05)
                    ser.reset_input_buffer()
                    ser.write(frame_feedback(MOTOR_ID))
                    time.sleep(0.2)
                    resp = ser.read_all()
                    if resp and len(resp) >= 10:
                        fb = decode_feedback(resp[:10])
                        if fb:
                            print(f"  Mode:     {fb['mode']}")
                            print(f"  Speed:    {fb['speed_rpm']} RPM")
                            print(f"  Current:  {fb['current_a']} A")
                            print(f"  Position: {fb['position_deg']}°")
                            print(f"  Temp:     {fb['temp_c']} °C")
                            print(f"  Error:    {fb['error']}")
                    else:
                        print("  ✗ No feedback received")
                    state["polling"] = True

                elif key == 'Q':
                    state["frame"]   = frame_velocity(MOTOR_ID, 0)
                    time.sleep(0.1)
                    for _ in range(5):
                        ser.write(frame_brake(MOTOR_ID))
                        time.sleep(0.02)
                    state["running"] = False
                    break

    except KeyboardInterrupt:
        print("\n[!] Interrupted")
    finally:
        state["running"] = False
        time.sleep(0.1)
        ser.close()
        print("[✓] Port closed. Goodbye!")


if __name__ == "__main__":
    main()