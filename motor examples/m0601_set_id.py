"""
m0601_set_id.py — Change the RS485 ID of one M0601 motor.

⚠  Connect only ONE motor to the bus before running.
⚠  The motor saves the ID when powered off (non-volatile).

Usage:
    pip install pyserial
    python m0601_set_id.py
"""

import serial
import time
import sys

COM_PORT     = "COM13"       # ← Change to your port
BAUD_RATE    = 115200
NEW_MOTOR_ID = 0x01          # ← Change to the ID you want to assign (0x01–0xFE)


def crc8_maxim(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if (crc & 0x01) else (crc >> 1)
    return crc


# Verified broadcast ID query frame
FRAME_ID_QUERY = bytes([0xC8, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE])


def build_id_set_frame(new_id):
    """
    ID set frame: AA 55 53 [ID] 00 00 00 00 00 00
    Note: This frame has NO CRC — the last byte is always 0x00.
    Must be sent exactly 5 times in a row.
    """
    assert 0x01 <= new_id <= 0xFE, "ID must be 0x01 to 0xFE"
    return bytes([0xAA, 0x55, 0x53, new_id, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])


def query_current_id(ser):
    """Send broadcast query and return the detected motor ID, or None."""
    ser.reset_input_buffer()
    ser.write(FRAME_ID_QUERY)
    time.sleep(0.3)
    resp = ser.read_all()
    if not resp or bytes(resp) == FRAME_ID_QUERY:
        return None
    # Motor ID is the first non-echo byte in the 0x01–0xFE range
    for byte in resp:
        if 0x01 <= byte <= 0xFE:
            return byte
    return None


def main():
    print("=" * 52)
    print("  M0601 Motor ID Changer")
    print(f"  Port: {COM_PORT}  →  New ID: 0x{NEW_MOTOR_ID:02X} ({NEW_MOTOR_ID})")
    print("=" * 52)
    print()
    print("  ⚠  ONLY ONE MOTOR MUST BE CONNECTED TO THE BUS.")
    print("  ⚠  The new ID is saved permanently (survives power-off).")
    print()

    try:
        ser = serial.Serial(
            port=COM_PORT, baudrate=BAUD_RATE,
            bytesize=8, parity='N', stopbits=1, timeout=0.3
        )
        print(f"[✓] Opened {COM_PORT}")
    except serial.SerialException as e:
        print(f"[✗] Cannot open port: {e}")
        sys.exit(1)

    # Step 1: Confirm one motor is present and read current ID
    print("\n[Step 1] Scanning for motor...")
    current_id = query_current_id(ser)
    if current_id is None:
        print("[✗] No motor detected.")
        print("    Check: 18V power ON? Wiring correct? Brown wire to GND?")
        ser.close()
        sys.exit(1)

    print(f"[✓] Motor detected — current ID: 0x{current_id:02X} ({current_id})")

    if current_id == NEW_MOTOR_ID:
        print(f"[!] Motor already has ID 0x{NEW_MOTOR_ID:02X}. Nothing to do.")
        ser.close()
        sys.exit(0)

    # Step 2: Confirm with user
    print(f"\n[Step 2] Ready to change ID: 0x{current_id:02X} → 0x{NEW_MOTOR_ID:02X}")
    confirm = input("  Type 'yes' to confirm: ").strip().lower()
    if confirm != 'yes':
        print("[!] Cancelled.")
        ser.close()
        sys.exit(0)

    # Step 3: Send ID set frame exactly 5 times
    print("\n[Step 3] Sending ID set frame ×5...")
    id_frame = build_id_set_frame(NEW_MOTOR_ID)
    print(f"  Frame: {id_frame.hex(' ').upper()}")

    for i in range(1, 6):
        ser.write(id_frame)
        print(f"  Sent {i}/5")
        time.sleep(0.05)   # Small gap between frames

    print("\n[Step 4] Waiting for motor to save ID (power-cycle not needed)...")
    time.sleep(0.5)

    # Step 4: Verify new ID
    print("[Step 4] Verifying new ID...")
    verified_id = query_current_id(ser)

    if verified_id == NEW_MOTOR_ID:
        print(f"\n[✓] SUCCESS — Motor ID is now 0x{NEW_MOTOR_ID:02X} ({NEW_MOTOR_ID})")
        print(f"    Update MOTOR_ID = 0x{NEW_MOTOR_ID:02X} in your other scripts.")
    elif verified_id is not None:
        print(f"\n[✗] Motor responded with ID 0x{verified_id:02X} — ID change may have failed.")
        print("    Try power-cycling the motor and running this script again.")
    else:
        print("\n[?] No response after ID change.")
        print("    Try power-cycling the motor — the new ID may have been saved.")
        print(f"    Run m0601_scan.py after power-cycling to confirm.")

    ser.close()
    print("[✓] Port closed.")


if __name__ == "__main__":
    main()