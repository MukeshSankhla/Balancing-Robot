"""
m0601_scan.py — Discover all M0601 motors on the RS485 bus.

Stage 1: Broadcast ID query (fast, ~0.3s)
Stage 2: Full poll of all IDs 0x01-0xFE (thorough, ~40s at 0.15s timeout)

Usage:
    pip install pyserial
    python m0601_scan.py
"""

import serial
import time
import sys

COM_PORT  = "COM13"       # ← Change to your port (e.g. /dev/ttyUSB0 on Linux)
BAUD_RATE = 115200
TIMEOUT   = 0.15          # Seconds to wait per ID. Increase to 0.25 on noisy buses.


def crc8_maxim(data):
    """CRC-8/MAXIM: polynomial x^8 + x^5 + x^4 + 1, reflected 0x8C."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if (crc & 0x01) else (crc >> 1)
    return crc


# --- Verified frames ---
FRAME_ID_QUERY = bytes([0xC8, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE])

def frame_feedback_request(motor_id):
    """Build a feedback query frame for a specific motor ID."""
    f = [motor_id, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    f.append(crc8_maxim(f))
    return bytes(f)


def stage1_broadcast(ser):
    """
    Send the broadcast ID query and parse the response.
    The motor replies with its ID in DATA[0] of the response frame.
    Returns detected ID (int) or None.
    """
    print("\n[Stage 1] Broadcast ID query...")
    ser.reset_input_buffer()
    ser.write(FRAME_ID_QUERY)
    time.sleep(0.3)

    resp = ser.read_all()
    if not resp:
        print("  No response.")
        return None

    print(f"  Raw response: {resp.hex(' ').upper()}")

    # If response is identical to our query, it's just the RS485 echo — not a motor reply.
    if bytes(resp) == FRAME_ID_QUERY:
        print("  (Echo only - motor did not respond separately)")
        return None

    # Motor reply starts with its own ID byte (0x01–0xFE)
    for byte in resp:
        if 0x01 <= byte <= 0xFE:
            print(f"  [+] Motor found via broadcast - ID: 0x{byte:02X} ({byte})")
            return byte

    return None


def stage2_full_poll(ser):
    """
    Poll each ID from 0x01 to 0xFE individually.
    Returns a list of all responding IDs.
    """
    print("\n[Stage 2] Full ID poll (0x01 -> 0xFE)...")
    print(f"  Estimated time: {254 * TIMEOUT:.0f}s - please wait.\n")
    found = []

    for motor_id in range(0x01, 0xFF):
        # Print progress bar
        pct   = motor_id / 254
        bar   = "#" * int(30 * pct) + "-" * (30 - int(30 * pct))
        print(f"\r  [{bar}] 0x{motor_id:02X} ({motor_id}/254)", end="", flush=True)

        ser.reset_input_buffer()
        ser.write(frame_feedback_request(motor_id))
        time.sleep(TIMEOUT)

        resp = ser.read_all()
        # A valid reply is 10 bytes starting with the motor's own ID
        if resp and len(resp) >= 3 and resp[0] == motor_id:
            found.append(motor_id)
            print(f"\n  [+] Motor at ID 0x{motor_id:02X} ({motor_id}) replied!")

    print(f"\r  [{'#' * 30}] Done!                                  ")
    return found


def main():
    print("=" * 52)
    print("  M0601 Motor Scanner")
    print(f"  Port: {COM_PORT}  |  Baud: {BAUD_RATE}")
    print("=" * 52)
    print("\n  [!] For Stage 1, ensure only ONE motor is on the bus.")
    print("  Press Enter to start scan, Ctrl+C to cancel...")
    input()

    try:
        ser = serial.Serial(
            port=COM_PORT, baudrate=BAUD_RATE,
            bytesize=8, parity='N', stopbits=1,
            timeout=TIMEOUT
        )
        print(f"[+] Opened {COM_PORT}")
    except serial.SerialException as e:
        print(f"[-] Cannot open port: {e}")
        sys.exit(1)

    try:
        broadcast_id = stage1_broadcast(ser)
        polled_ids   = stage2_full_poll(ser)

        # Combine results
        all_ids = sorted(set(polled_ids) | ({broadcast_id} if broadcast_id else set()))

        print("\n" + "=" * 52)
        print("  SCAN COMPLETE")
        print("=" * 52)
        if all_ids:
            print(f"\n  {len(all_ids)} motor(s) found:")
            for mid in all_ids:
                print(f"    - ID 0x{mid:02X}  (decimal {mid})")
            if len(all_ids) == 1:
                print(f"\n  -> In other scripts, set:  MOTOR_ID = 0x{all_ids[0]:02X}")
        else:
            print("\n  [-] No motors detected.")
            print("  Checklist:")
            print("    1. Is the 18V power adapter ON?")
            print("    2. Is the Brown wire connected to GND?")
            print("    3. Try swapping Orange <-> White (A/B polarity)")
            print(f"    4. Try increasing TIMEOUT (currently {TIMEOUT}s)")
        print("=" * 52)

    except KeyboardInterrupt:
        print("\n[!] Scan cancelled.")
    finally:
        ser.close()
        print("[+] Port closed.")


if __name__ == "__main__":
    main()