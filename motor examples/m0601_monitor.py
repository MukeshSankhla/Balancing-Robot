"""
m0601_monitor.py — Live parameter monitoring with CSV logging.

Polls the motor feedback at configurable interval and displays:
  Mode | Speed (RPM) | Current (A) | Position (°) | Temp (°C) | Errors

Also writes all readings to motor_log.csv for offline analysis.

Usage:
    pip install pyserial
    python m0601_monitor.py
    Press Ctrl+C to stop.
"""

import serial
import time
import sys
import csv
import os
from datetime import datetime

COM_PORT      = "COM13"     # ← Change to your port
BAUD_RATE     = 115200
MOTOR_ID      = 0x01
POLL_INTERVAL = 0.2         # Seconds between feedback queries (5Hz default)
LOG_FILE      = "motor_log.csv"


def crc8_maxim(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if (crc & 0x01) else (crc >> 1)
    return crc


def frame_feedback_query(motor_id):
    """Protocol 2 feedback request: returns speed, current, temp, position, error."""
    f = [motor_id, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    f.append(crc8_maxim(f))
    return bytes(f)


def decode_error(code):
    """Decode error bitmask into a human-readable string."""
    if code == 0:
        return "OK"
    errors = []
    if code & 0x01: errors.append("SensorErr")
    if code & 0x02: errors.append("Overcurrent")
    if code & 0x04: errors.append("PhaseOvercurrent")
    if code & 0x08: errors.append("Stall")
    if code & 0x10: errors.append("Troubleshoot")
    return " | ".join(errors)


def parse_feedback(data):
    """
    Parse a 10-byte Protocol-2 feedback frame.
    Returns a dict with all motor parameters, or None on invalid data.
    """
    if len(data) < 10:
        return None

    motor_id = data[0]
    mode_raw = data[1]
    mode_map = {0x01: "Current ", 0x02: "Velocity", 0x03: "Position"}
    mode_str = mode_map.get(mode_raw, f"Unk(0x{mode_raw:02X})")

    # Torque current: signed INT16, big-endian, bytes 2-3
    raw_current = int.from_bytes(data[2:4], byteorder='big', signed=True)
    current_a   = round(raw_current * 8.0 / 32767.0, 3)

    # Velocity: signed INT16, big-endian, bytes 4-5
    speed_rpm = int.from_bytes(data[4:6], byteorder='big', signed=True)

    # Winding temperature: uint8, byte 6
    temp_c = data[6]

    # U8 position: uint8, byte 7 — 0-255 maps to 0°-360°
    u8_pos       = data[7]
    position_deg = round(u8_pos * 360.0 / 255.0, 1)

    # Error code bitmask: byte 8
    error_raw = data[8]
    error_str = decode_error(error_raw)

    return {
        "timestamp":    datetime.now().strftime("%H:%M:%S.%f")[:-3],
        "motor_id":     motor_id,
        "mode":         mode_str,
        "speed_rpm":    speed_rpm,
        "current_a":    current_a,
        "temp_c":       temp_c,
        "position_deg": position_deg,
        "error_code":   f"0x{error_raw:02X}",
        "error_str":    error_str,
        "raw_hex":      data[:10].hex(' ').upper(),
    }


def print_dashboard(fb, count):
    """Print a compact, self-overwriting dashboard line."""
    fault_indicator = "🔴 FAULT" if fb['error_str'] != "OK" else "🟢 OK   "
    line = (
        f"\r[{fb['timestamp']}] #{count:5d} | "
        f"Mode: {fb['mode']} | "
        f"Speed: {fb['speed_rpm']:+5d} RPM | "
        f"Current: {fb['current_a']:+6.3f} A | "
        f"Pos: {fb['position_deg']:6.1f}° | "
        f"Temp: {fb['temp_c']:3d}°C | "
        f"{fault_indicator}"
        + (f" ({fb['error_str']})" if fb['error_str'] != "OK" else "")
    )
    print(line, end="", flush=True)


def main():
    print("=" * 70)
    print("  M0601 Motor Parameter Monitor")
    print(f"  Port: {COM_PORT}  |  Motor ID: 0x{MOTOR_ID:02X}  |  Poll: {POLL_INTERVAL*1000:.0f}ms")
    print(f"  Logging to: {os.path.abspath(LOG_FILE)}")
    print("=" * 70)
    print("  Press Ctrl+C to stop.\n")

    try:
        ser = serial.Serial(
            port=COM_PORT, baudrate=BAUD_RATE,
            bytesize=8, parity='N', stopbits=1, timeout=0.3
        )
        print(f"[✓] Opened {COM_PORT}")
    except serial.SerialException as e:
        print(f"[✗] Cannot open port: {e}")
        sys.exit(1)

    query_frame = frame_feedback_query(MOTOR_ID)
    count        = 0
    no_resp_count = 0

    # Open CSV log file
    csv_fields = [
        "timestamp", "motor_id", "mode", "speed_rpm",
        "current_a", "temp_c", "position_deg",
        "error_code", "error_str", "raw_hex"
    ]
    log_file = open(LOG_FILE, 'w', newline='')
    writer   = csv.DictWriter(log_file, fieldnames=csv_fields)
    writer.writeheader()

    print(f"\n{'Timestamp':14s} {'#':>6s}  Mode      Speed    Current  Position  Temp  Status")
    print("-" * 80)

    try:
        while True:
            ser.reset_input_buffer()
            ser.write(query_frame)
            time.sleep(POLL_INTERVAL)

            resp = ser.read_all()
            if not resp or len(resp) < 10:
                no_resp_count += 1
                if no_resp_count >= 5:
                    print(f"\r[!] No response for {no_resp_count} consecutive polls — check motor power.", end="")
                continue

            no_resp_count = 0
            count += 1
            fb = parse_feedback(resp[:10])
            if not fb:
                continue

            print_dashboard(fb, count)
            writer.writerow({k: fb[k] for k in csv_fields})
            log_file.flush()

    except KeyboardInterrupt:
        print("\n\n[✓] Monitoring stopped.")
    finally:
        log_file.close()
        ser.close()
        print(f"[✓] Log saved to {LOG_FILE}")
        print(f"[✓] {count} readings recorded.")
        print("[✓] Port closed.")


if __name__ == "__main__":
    main()