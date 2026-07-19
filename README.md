# Self-Balancing Robot Controller (M0601 & ESP32 MPU-6050)

This directory contains the files required to build a self-balancing two-wheeled robot using **DFRobot M0601 Direct-Drive Hub Motors** and an **MPU-6050 IMU** connected to an **ESP32**.

## System Overview

```
                      +-------------------+
                      |   MPU-6050 IMU    | (Measures tilt angle & angular velocity)
                      +---------+---------+
                                | I2C (SDA=18, SCL=19)
                                v
                      +---------+---------+
                      |       ESP32       | (UART TX=22, RX=21)
                      +---------+---------+
                                | TTL UART (Telemetry Stream @ 100Hz)
                                v
                      +---------+---------+
                      |     Laptop/PC     | (COM10 receives telemetry, COM13 controls motors)
                      +---------+---------+
                                | USB RS485 Dongle
                                v
                 +--------------+--------------+
                 |                             |
                 v                             v
       +---------+---------+         +---------+---------+
       | Right Motor (0x01)|         | Left Motor (0x02) |
       +-------------------+         +-------------------+
```

---

## 1. Hardware Pinout & Wiring

Ensure your system is wired correctly before powering it on.

### MPU-6050 to ESP32 (I2C)
| MPU-6050 Pin | ESP32 GPIO | Description |
| :--- | :--- | :--- |
| **VCC** | **3.3V** | Power |
| **GND** | **GND** | Ground reference |
| **SDA** | **GPIO 18** | I2C Data line |
| **SCL** | **GPIO 19** | I2C Clock line |

### ESP32 to USB-to-TTL Serial Adapter (COM10 UART)
| ESP32 Pin | TTL Adapter Pin | Description |
| :--- | :--- | :--- |
| **GPIO 22 (TX)** | **RXD** | Data transmission from ESP32 |
| **GPIO 21 (RX)** | **TXD** | Data reception by ESP32 (Optional) |
| **GND** | **GND** | Ground Reference |

### Motors to RS485 Bus & PC
- Connect the **A (+)** wires (White) of both Right (0x01) and Left (0x02) motors to the **A (+)** pin of your USB-to-RS485 adapter (COM13).
- Connect the **B (-)** wires (Orange) of both motors to the **B (-)** pin of the adapter.
- Connect the **GND** wires (Black/Brown) of both motors to the **GND** pin of the adapter.
- Power both motors using a stable **12V - 18V DC power supply** (Red wire to positive, Black wire to ground).

---

## 2. Flashing the ESP32 Firmware

1. Open Arduino IDE.
2. Go to **File -> Open** and select the [esp32_mpu_stream.ino](file:///c:/Users/mukes/Downloads/MotorLink/balance_robot/esp32_mpu_stream/esp32_mpu_stream.ino) sketch.
3. Select your ESP32 board in the board manager (e.g. **ESP32 Dev Module**).
4. Select the corresponding Port.
5. Click **Upload**.
6. Open the **Serial Monitor** or connect your TTL reader (COM10) at **115200 baud** to verify functionality:
   - Ensure the robot is **completely still** during startup so the gyro offset calibration succeeds.
   - You should see the streaming output in the format: `PITCH:<val>,ROLL:<val>,GYRO_Y:<val>,GYRO_X:<val>`.

---

## 3. Python Setup & Execution

### Installation
Ensure you have the required dependencies installed:
```bash
pip install pyserial
```

### Running the Balancing Controller
Run the script. By default, it uses `COM10` for ESP32 and `COM13` for motors.

**Example (Windows - using defaults):**
```bash
python balance_robot.py --kp 6.5 --kd 0.15 --ki 0.2
```

**Example (Windows - overriding ports):**
```bash
python balance_robot.py --esp COM10 --motor COM13 --kp 6.5 --kd 0.15 --ki 0.2
```

**Example (Linux/macOS):**
```bash
python balance_robot.py --esp /dev/ttyUSB1 --motor /dev/ttyUSB0 --kp 6.5 --kd 0.15 --ki 0.2
```

### Parameter Reference
| Argument | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `--esp` | String | `COM10` | ESP32 serial port identifier. |
| `--motor` | String | `COM13` | USB RS485 adapter serial port identifier. |
| `--kp` | Float | `1.2` | Proportional gain. Controls restoration force based on tilt. |
| `--ki` | Float | `0.0` | Integral gain. Eliminates steady-state lean. |
| `--kd` | Float | `0.03` | Derivative gain. Dampens oscillations (using direct gyro rate). |
| `--target` | Float | `0.0` | Balancing angle offset (in degrees) to calibrate the weight center. |
| `--limit` | Int | `50` | Safety speed limit (RPM) allowed for wheels. |
| `--safety-angle`| Float | `35.0` | Maximum tilt angle allowed before emergency shutoff. |
| `--right-sign` | Int | `-1` | Direction multiplier for the Right motor (`1` or `-1`). |
| `--left-sign` | Int | `1` | Direction multiplier for the Left motor (`1` or `-1`). |
| `--show-raw`  | Flag  | `False` | Prints raw accelerometer (X,Y,Z in G) and gyroscope (X,Y,Z in deg/s) values instead of default dashboard. |
| `--slew-limit`| Float | `150.0` | Max rate of change in RPM per second allowed. Restricts sudden motor acceleration for smoother balance. |
| `--kp-nonlin` | Float | `0.05` | Non-linear Proportional gain coefficient. Dynamically scales Kp up for larger tilt errors to prevent falls. |
| `--kp-vel`    | Float | `0.01` | Velocity loop proportional feedback gain. Helps slow down the robot if it drifts forward/backward. |
| `--ki-vel`    | Float | `0.0005`| Velocity loop integral feedback gain. Automatically balances COG offset to prevent runaway/drift. |

---

## 4. PID Tuning Walkthrough

Tuning a self-balancing robot is a systematic process. Follow these steps to achieve stable balancing:

### Step 1: Direction Verification (Safety First!)
1. Lift the robot off the ground (wheels in the air).
2. Run the script with small gains: `python balance_robot.py --esp <PORT> --motor <PORT> --kp 2.0 --kd 0.0 --ki 0.0`.
3. Tilt the robot **forward** (pitch increases).
   - Verify that **both wheels rotate in the forward direction** (trying to drive underneath the tilt).
4. Tilt the robot **backward** (pitch decreases).
   - Verify that **both wheels rotate in the backward direction**.
5. *If a wheel rotates the wrong way*, adjust the corresponding direction signs using `--right-sign` or `--left-sign` (e.g. flip `1` to `-1` or vice-versa).

### Step 2: Establish the Balancing Point (`--target`)
The physical balancing point of your robot might not correspond to exactly $0.0^{\circ}$ due to sensor mounting inaccuracies or weight distribution:
1. Hold the robot at its absolute physical balance point (where it doesn't want to fall to either side).
2. Note the average pitch angle printed on the terminal.
3. Pass this value as the `--target` parameter (e.g. `--target -1.5` if it balances at $-1.5^{\circ}$).

### Step 3: Tuning Proportional Gain (`Kp`)
1. Set `Kd = 0` and `Ki = 0`.
2. Gradually increase `Kp` (starting from `1.0`, e.g. `2.0`, `4.0`, `6.0`...) and place the robot on the floor.
3. Keep increasing `Kp` until the robot starts to resist falling and oscillates back and forth rapidly around the balance point.

### Step 4: Tuning Derivative Gain (`Kd`)
1. With your active `Kp`, begin introducing `Kd` (start small, e.g. `0.05`, `0.1`, `0.15`...).
2. `Kd` acts as a dampener. Increase it until the rapid oscillations caused by `Kp` are suppressed, and the robot stands up relatively smoothly.

### Step 5: Tuning Integral Gain (`Ki`)
1. If the robot balances but slowly drifts in one direction, or stays balanced but at a constant lean, introduce a small amount of `Ki` (start at `0.05` to `0.2`).
2. The integral term will accumulate the error over time and force the robot back to the target angle. Keep it low to prevent low-frequency wobbling.
