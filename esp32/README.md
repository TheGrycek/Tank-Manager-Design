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
| `history.py` | `History`: level history for the app |
| `mpu9250.py`, `mpu6500.py`, `ak8963.py` | MPU9250 driver |
| `sh1106.py` | OLED driver |
| `aioble/` | Bluetooth LE library |

Files created on the device: `settings.json` (user settings), `mpu_calibration.json`
(MPU calibration), `ble_secrets.json` (Bluetooth bonds) and `history.bin` (level history, 35 KB).

## Components

| Component | Class | Pins | Notes |
| --- | --- | --- | --- |
| Gray tank level sensor | `GrayLevelSensor` | ADC 34 | Float sensor IG-0190FM, 0 Ω empty to 186 Ω full, 200 Ω divider to 3.3 V |
| Clean tank level sensor | `CleanLevelSensor` | ADC 35 | Float sensor IG-0190FM, 0 Ω empty to 186 Ω full, 200 Ω divider to 3.3 V |
| Gray tank temperature sensor | `TemperatureSensor` | ADC 32 | NTC 10k, 10 kΩ divider to 3.3 V |
| MPU9250 (accelerometer, magnetometer) | `MPU` | I2C: SCL 22, SDA 21 | Pitch, roll and yaw of the device |
| OLED 128x64, SH1106 | `OLED` | I2C: SCL 18, SDA 19 | Address 0x3C |
| Electric valve | `Valve` | open 13, close 27, current ADC 33 | Moves for 35 s, closes by itself 10 min after opening and after every start-up; the motor current (1 Ω shunt, 1 mV = 1 mA) is logged while moving |
| Heater | `Heater` | 25 | On at the heating threshold, off 1 °C above it |
| Buzzer | `Buzzer` | 26 | Gray tank overfill alarm, a double beep every second; in the clean tank fill mode beeps that come faster as the tank fills (silent up to 5 %, continuous from 95 %) |
| EC12 rotary encoder with button | `Encoder` | A 17, B 23, SW 5 | Internal pull-ups, active low |

The level is mapped linearly between the sensor resistance with the tank empty and full
(`LEVEL_R_EMPTY`, `LEVEL_R_FULL` in `sensors.py`: 0 Ω and the measured 186 Ω). The voltage on the
ADC is then 0-1.6 V.
The float (30 cm sensors) does not reach the bottom of the tank, so the percentage is that of the
float travel, not of the tank volume: 0 % means the float rests at its lowest point and some water
is still left below it.

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
| `FILL MODE` | Clean tank fill mode on / off (not kept after a restart, off by itself after 30 min) |
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
| MPU | `...0004` | 3 x `float32`: pitch (> 0 front up), roll (> 0 right side down), heading of the front (0-360, clockwise from magnetic north), degrees | read, notify |
| Valve | `...0005` | `uint8`, 1 = open, 0 = closed | read, write, notify, indicate |
| Valve state | `...0006` | `uint8`, 0 = closed, 1 = open, 2 = closing, 3 = opening | read, notify, indicate |
| Heater | `...0007` | `uint8`, 1 = on | read, notify, indicate |
| Settings | `...0008` | 5 x `uint8`: buzzer, gray level threshold, auto-heating, heating threshold, fill mode (the app may write only the first 4) | read, write, notify |
| History | `...0009` | written: `<I` first sample wanted, `<I` Unix time; notified: `<I` first sample, `uint8` count, count x (`<I` time, `uint8` gray (bit 7: driving), `uint8` clean (bit 7: tilted), `<h` temperature x 100), a packet with count 0 ends a transfer | write, notify |

### History

`History` stores one sample every 10 minutes (the average of the readings in that time: both
levels and the gray tank temperature) in `history.bin`, a ring buffer of 30 days (4320 samples of
8 bytes). Samples are numbered, so the app downloads only the ones it does not have: it writes the
first number it lacks together with the current time, and the controller answers with packets of
up to 29 samples (the app asks for an ATT MTU of 247; with the default MTU one sample per packet),
then an end packet. Samples taken later during the connection are pushed as they come. If the app
asks for a number the controller does not have yet (history file recreated), everything is sent.

The controller does not know the date: it learns it from the app's request. Samples taken before
that carry the seconds since start-up with the top bit of the time set; the ones of the current
start-up are dated when the time arrives, older ones (from before a restart without a phone) stay
undated and the app skips them. Every app connection sends the time; after the first one only
corrections of up to 10 min are taken (clock drift), so a phone or tablet with a wrong clock cannot
misdate the samples.

While driving the water sloshes and on a slope it lies at a slant, so the level sensors read wrong.
A sample is flagged as driving (top bit of the gray level byte) when the acceleration changed by
more than 1 m/s^2 between two MPU readings (1 s apart) at least 5 times in it, and as tilted (top
bit of the clean level byte) when the average pitch or roll was beyond 3 degrees
(`History.MOTION_DELTA`, `MOTION_COUNT`, `TILT_DEG`; to be tuned on the road). The app draws such
samples faint and leaves them out of its statistics. Without a calibrated MPU nothing is flagged.

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
| `failed: I2C error N` | Reading the sensor failed (N is the error number, e.g. 19 = no answer, 116 = timeout) |
| `failed: saving failed` | Writing `mpu_calibration.json` failed |
| `MPU not found` | The sensor (or its magnetometer) does not answer on I2C; it is set up again at the start of every calibration |

### Stored result

The result is saved to `mpu_calibration.json` and loaded on every start, so it survives a
restart:

```json
{"mag_offset": [x, y, z], "mag_scale": [x, y, z], "acc_bias": [x, y, z], "camper_level": [pitch, roll]}
```

`mag_offset` is in µT, `acc_bias` in m/s² and `camper_level` in degrees. The file exists only on the device: erasing the
flash removes the calibration, copying the `.py` files does not.

### Mounting orientation

The calibration is done lying flat (sensor Z up) and its values are measured and applied in the
sensor's own axes, so they do not depend on the mounting.

The angles are computed in the camper's axes: x forward, y to the left side, z up. On the front
panel the sensor is mounted with Z towards the rear of the camper, Y towards the floor and X
towards the left side (`MPU._to_camper()`; change it there if the mounting changes).
`MPU.get_rotation()` returns:

- pitch: > 0 when the front is higher than the rear,
- roll: > 0 when the right side is lower than the left,
- heading: where the front points, 0-360° clockwise from magnetic north (not true north),
  tilt-compensated with the accelerometer.

Pitch and roll are relative to the level set with `LEVEL` (below); the heading is not.

### Level

The mounting is never perfectly square, and the ground used in step 3 of the calibration is
never perfectly level (its tilt ends up in the accelerometer bias). To correct both, park the
vehicle level after the device is mounted, open the menu, turn to `LEVEL` and press the encoder:
the current pitch and roll are stored as zero and subtracted from the reported angles. Yaw is
not affected. The level is kept when the MPU is calibrated again; it needs a calibrated MPU.
A level saved by older firmware (key `level`, other axes) is ignored: set it again.

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
