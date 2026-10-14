# ESP32 firmware

MicroPython firmware of the tank manager: it reads the tank level, temperature and tilt sensors,
drives the gray water valve, the heater and the buzzer, shows the state on an OLED display and
talks to the phone app over Bluetooth LE.

Runs on a generic ESP32 with MicroPython 1.25.

## Files

| File | Contents |
| --- | --- |
| `main.py` | Creates the components and runs the asyncio tasks |
| `sensors.py` | `LevelSensor`, `TemperatureSensor`, `MPU` |
| `peripherals.py` | `OLED`, `Valve`, `Heater`, `Buzzer`, `Encoder` |
| `setup.py` | `Settings`, `Menu`, `Bluetooth` |
| `mpu9250.py`, `mpu6500.py`, `ak8963.py` | MPU9250 driver |
| `sh1106.py` | OLED driver |
| `aioble/` | Bluetooth LE library |

Files created on the device: `settings.json` (user settings), `mpu_calibration.json`
(MPU calibration) and `ble_secrets.json` (Bluetooth bonds).

## Components

| Component | Class | Pins | Notes |
| --- | --- | --- | --- |
| Gray tank level sensor | `GrayLevelSensor` | ADC 34 | Resistive, 200 Ω divider to 3.3 V |
| Clean tank level sensor | `CleanLevelSensor` | ADC 35 | Resistive, 200 Ω divider to 3.3 V |
| Gray tank temperature sensor | `TemperatureSensor` | ADC 32 | NTC 10k, 10 kΩ divider to 3.3 V |
| MPU9250 (accelerometer, magnetometer) | `MPU` | I2C: SCL 22, SDA 21 | Pitch, roll and yaw of the device |
| OLED 128x64, SH1106 | `OLED` | I2C: SCL 18, SDA 19 | Address 0x3C |
| Electric valve | `Valve` | open 13, close 27 | Moves for 35 s, closes by itself 10 min after opening |
| Heater | `Heater` | 25 | On at the heating threshold, off 1 °C above it |
| Buzzer | `Buzzer` | 26 | Gray tank overfill alarm |
| EC12 rotary encoder with button | `Encoder` | A 17, B 23, SW 5 | Internal pull-ups, active low |

The level sensors are not calibrated yet: the resistances for an empty and a full tank
(`r_empty`, `r_full` in `sensors.py`) are placeholders.

### Display and menu

The display normally shows the tank levels, the temperature and the valve state. Pressing the
encoder opens the menu; turning selects an item and pressing changes it. The menu closes after
15 s without input.

| Item | Action |
| --- | --- |
| `BUZZER` | Overfill alarm on / off |
| `GRAY LVL THR` | Gray tank level that triggers the alarm, 50-100 % |
| `AUTO-HEATING` | Heater control on / off |
| `HEATING THR` | Temperature below which the heater turns on, 1-5 °C |
| `VALVE` | Open / close the valve |
| `MPU` | Shows `CALIBRATED` or `NOT CALIBRATED`; pressing starts the MPU calibration |
| `LEVEL` | Take the current pitch and roll as zero |

### Bluetooth

The device advertises as `Mati&Mati`. Pairing requires the 6-digit code shown on the display,
and all characteristics are available to authenticated connections only.
A paired phone is remembered (`ble_secrets.json`) and reconnects without a code. Phones connect
from a private address that changes every time; the controller does not resolve it under
MicroPython, so `Bluetooth._bonded_key()` resolves it with the phone's stored IRK.

Bluetooth is started at the top of `main.py`, before the other modules are imported: compiling
them first grows the Python heap into the memory the Bluetooth controller needs, and the board
then crashes at start-up (`BLE_INIT: hci inits failed`).

| Characteristic | UUID suffix | Format | Access |
| --- | --- | --- | --- |
| Clean tank level | `...0001` | `uint8`, % | read, notify |
| Gray tank level | `...0002` | `uint8`, % | read, notify |
| Gray tank temperature | `...0003` | `int16`, 0.01 °C, -32768 = sensor fault | read, notify |
| MPU | `...0004` | 3 x `float32`: pitch, roll, yaw in degrees | read, notify |
| Valve | `...0005` | `uint8`, 1 = open, 0 = closed | read, write, notify, indicate |
| Valve locked | `...0006` | `uint8`, always 0 | read, notify, indicate |
| Heater | `...0007` | `uint8`, 1 = on | read, notify, indicate |
| Settings | `...0008` | 4 x `uint8`: buzzer, gray level threshold, auto-heating, heating threshold | read, write, notify |

## MPU calibration

The MPU has to be calibrated once, with the device lying flat, before it is mounted. Until then
no angles are sent to the app. The calibration finds:

- the magnetometer hard and soft iron correction (offset and scale per axis),
- the accelerometer bias.

### Procedure

1. Open the menu, turn to `MPU` and press the encoder.
2. `Rotate the device` (step 1/2, 20 s countdown): turn the device in your hand around all three
   axes, e.g. in a figure of eight. Spinning it flat on the ground is not enough, every axis
   has to see the full range of the magnetic field.
3. `Lay down the device` (step 2/2): put the device flat on level ground. When it lies flat the
   display shows `hold still` and the bias is measured for 3 s. You have 60 s to lay it down.
4. `Calibration saved` stays on the display for 5 s, or until the encoder is pressed.

Pressing the encoder during the calibration cancels it. A cancelled or failed calibration keeps
the previous one.

| Message | Meaning |
| --- | --- |
| `failed: rotate more` | An axis of the magnetometer changed by less than 20 µT in step 1 |
| `failed: not laid flat` | The device was not flat and still for 3 s within 60 s |
| `failed: sensor error` | Reading the sensor or saving the file failed |
| `MPU not found` | The sensor does not answer on I2C |

### Stored result

The result is saved to `mpu_calibration.json` and loaded on every start, so it survives a
restart:

```json
{"mag_offset": [x, y, z], "mag_scale": [x, y, z], "acc_bias": [x, y, z], "level": [pitch, roll]}
```

`mag_offset` is in µT, `acc_bias` in m/s² and `level` in degrees. The file exists only on the device: erasing the
flash removes the calibration, copying the `.py` files does not.

### Mounting orientation

The calibration is done lying flat (sensor Z up), but the device is mounted upright, turned
90° around the sensor X axis: sensor Y points up, where Z was, and sensor Z points where -Y was.
`MPU.get_rotation()` undoes this rotation, so pitch and roll are 0 in the mounted position and
the angles are the ones the device would give lying flat. As a result, the angles of a device
lying flat on a desk are not meaningful.

The calibration values are measured and applied in the sensor's own axes, so they do not depend
on the mounting.

### Level

The mounting is never perfectly square, and the ground used in step 3 of the calibration is
never perfectly level (its tilt ends up in the accelerometer bias). To correct both, park the
vehicle level after the device is mounted, open the menu, turn to `LEVEL` and press the encoder:
the current pitch and roll are stored as zero and subtracted from the reported angles. Yaw is
not affected. The level is kept when the MPU is calibrated again; it needs a calibrated MPU.

## Development

Install the tools:

```sh
uv tool install mpremote
uv tool install esptool
```

The user needs access to the serial port (`sudo usermod -aG dialout $USER`, then log in again).

```sh
mpremote connect /dev/ttyUSB0 cp -r . :   # copy the firmware, run from this folder
mpremote connect /dev/ttyUSB0 reset       # restart with the new code
mpremote connect /dev/ttyUSB0 repl        # console
```

The hardware watchdog is off by default (`WATCHDOG_MS = 0` in `main.py`). It cannot be stopped
once started, so keep it at 0 while developing over the REPL and set it (e.g. 8000) on the
installed device.
