import asyncio
import json
import time
from math import log, pow, atan2, sqrt, degrees, cos, sin

from machine import Pin, I2C, ADC

from mpu9250 import MPU9250


class LevelSensor:
    """Resistive tank level sensor on the low side of a divider (ADC to GND), 200R to 3.3V"""
    def __init__(self, pin, r_empty, r_full):
        self._LVL_SENSOR_PIN = Pin(pin, Pin.IN)
        self._MAX_LVL = 100
        self._MAX_V = 3.3
        self._R_DIVIDER = 200.0  # fixed resistor to 3.3V
        self._R_EMPTY = r_empty
        self._R_FULL = r_full
        self.adc = None

    def configure(self):
        try:
            self.adc = ADC(self._LVL_SENSOR_PIN)
            self.adc.atten(ADC.ATTN_11DB)
            return self.adc
        except OSError:
            return None

    def get_level(self):
        # read_uv() applies the factory ADC calibration (the raw reading is not linear)
        v = self.adc.read_uv() / 1000000
        v = min(max(v, 0.0), self._MAX_V - 1e-6)
        r_sensor = self._R_DIVIDER * v / (self._MAX_V - v)
        level = (r_sensor - self._R_EMPTY) / (self._R_FULL - self._R_EMPTY) * self._MAX_LVL
        return int(min(max(level, 0), self._MAX_LVL))


class GrayLevelSensor(LevelSensor):
    """Gray water tank level sensor"""
    def __init__(self):
        # TODO: calibrate - sensor resistance with the tank empty / full (measure it)
        super().__init__(34, r_empty=0.0, r_full=190.0)


class CleanLevelSensor(LevelSensor):
    """Clean water tank level sensor"""
    def __init__(self):
        # TODO: calibrate - sensor resistance with the tank empty / full (measure it)
        super().__init__(35, r_empty=0.0, r_full=190.0)


class TemperatureSensor:
    """Gray water tank temperature sensor NTC10k"""
    INVALID = -32768  # sent over BLE (int16) when the reading is not plausible

    def __init__(self):
        self._MAX_V = 3.3
        self._NTC_A = 1.14782450e-03
        self._NTC_B = 2.31732771e-04
        self._NTC_C = 9.16549992e-08
        self._R_DIVIDER = 10000.0
        self._KELVINS_ZERO = -273.15
        # Outside this range the sensor is treated as broken (open or shorted NTC)
        self._MIN_TEMP = -40.0
        self._MAX_TEMP = 85.0
        self._TEMP_SENSOR_PIN = Pin(32, Pin.IN)
        self.adc = None

    def configure(self):
        try:
            self.adc = ADC(self._TEMP_SENSOR_PIN)
            self.adc.atten(ADC.ATTN_11DB)
            return self.adc
        except OSError:
            return None

    def get_temperature(self):
        """Temperature in Celsius, or None when the sensor reading is not plausible"""
        if self.adc is None:
            return None
        # NTC is on the low side of the divider (ADC to GND), fixed resistor to 3.3V
        Vntc = self.adc.read_uv() / 1000000
        Vntc = min(max(Vntc, 1e-6), self._MAX_V - 1e-6)
        Rntc = self._R_DIVIDER * Vntc / (self._MAX_V - Vntc)
        temp_k = (1 / (self._NTC_A + (self._NTC_B * log(Rntc)) + (self._NTC_C * pow(log(Rntc), 3.0))))
        temp = temp_k + self._KELVINS_ZERO
        if temp < self._MIN_TEMP or temp > self._MAX_TEMP:
            return None
        return temp


class MPU:
    """MPU 9250 data calibration and retrieval"""
    _CALIBRATION_FILE = "mpu_calibration.json"
    _GRAVITY = 9.80665  # m/s^2, the value the MPU6500 driver scales with
    _ROTATE_MS = 20000  # magnetometer step: time to rotate the device around all axes
    _MAG_MIN_RANGE = 20.0  # uT, every axis must swing at least this much while rotating
    _FLAT_TIMEOUT_MS = 60000  # accelerometer step: time to lay the device down
    _FLAT_HOLD_MS = 3000  # and how long it must then stay flat
    _FLAT_MAX_ACC = 1.5  # m/s^2 on X and Y, above this the device is not lying flat
    _LEVEL_SAMPLES = 16  # accelerometer readings averaged when the level is set

    def __init__(self):
        self._MPU_I2C_SCL_PIN = Pin(22)
        self._MPU_I2C_SDA_PIN = Pin(21)
        self._MPU_FREQ = 100000
        self.i2c = None
        self.mpu = None
        self._acc_bias = (0.0, 0.0, 0.0)
        # Pitch and roll (degrees) of the mounted device with the vehicle standing level, set from the menu
        self._level = (0.0, 0.0)
        # False until a calibration is loaded from the file or done from the menu: angles are not reported then
        self.calibrated = False
        # True while the calibration runs: readings are raw then and must not be used
        self.calibrating = False
        # What the display shows during the calibration: (step, line, line, line)
        self.calibration_screen = ("", "", "", "")

    def configure(self):
        try:
            self.i2c = I2C(
                0,
                scl=self._MPU_I2C_SCL_PIN,
                sda=self._MPU_I2C_SDA_PIN,
                freq=self._MPU_FREQ
            )
            self.mpu = MPU9250(self.i2c)
            print("MPU9250 id: " + hex(self.mpu.whoami))
            self._load_calibration()
            return self.mpu
        except OSError:
            print("Error: MPU not found")
            self.mpu = None
            return None

    def _load_calibration(self):
        # Magnetometer hard/soft iron correction, in the units the driver returns (uT)
        try:
            with open(self._CALIBRATION_FILE) as f:
                data = json.load(f)
            self.mpu.ak8963._offset = tuple(data["mag_offset"])
            self.mpu.ak8963._scale = tuple(data["mag_scale"])
            # Accelerometer bias (m/s^2), missing in files saved by the magnetometer-only calibration
            self._acc_bias = tuple(data.get("acc_bias", (0.0, 0.0, 0.0)))
            self._level = tuple(data.get("level", (0.0, 0.0)))
            self.calibrated = True
            print("MPU calibration loaded")
        except (OSError, ValueError, KeyError):
            self.calibrated = False
            print("MPU calibration not found - angles are not reported, calibrate from the MPU menu")

    def _save_calibration(self):
        ak = self.mpu.ak8963
        with open(self._CALIBRATION_FILE, "w") as f:
            json.dump({"mag_offset": ak._offset, "mag_scale": ak._scale, "acc_bias": self._acc_bias,
                       "level": self._level}, f)

    def start_calibration(self):
        """Starts the calibration in the background (the panel menu); returns its task.
        The device is rotated in the hand first and then laid flat on the ground."""
        self.calibrating = True
        self.calibration_screen = ("1/2", "Rotate the", "device", "")
        return asyncio.create_task(self._calibrate())

    async def _calibrate(self):
        if self.mpu is None:
            self.calibration_screen = ("", "MPU not found", "", "")
            self.calibrating = False
            return
        ak = self.mpu.ak8963
        old_mag = (ak._offset, ak._scale)
        old_acc_bias = self._acc_bias
        error = "sensor error"
        try:
            # Raw magnetometer readings while measuring
            ak._offset = (0, 0, 0)
            ak._scale = (1, 1, 1)
            mag = await self._calibrate_magnetometer()
            if mag is None:
                error = "rotate more"
            else:
                acc_bias = await self._calibrate_accelerometer()
                if acc_bias is None:
                    error = "not laid flat"
                else:
                    ak._offset, ak._scale = mag
                    self._acc_bias = acc_bias
                    self._save_calibration()
                    self.calibrated = True
                    print("MPU calibration saved:", mag, acc_bias)
                    error = None
        except OSError as e:
            print("MPU calibration error:", e)
        finally:
            # Also when cancelled from the menu: keep the previous calibration
            if error is not None:
                ak._offset, ak._scale = old_mag
                self._acc_bias = old_acc_bias
            self.calibrating = False
        if error is None:
            self.calibration_screen = ("", "Calibration", "saved", "")
        else:
            self.calibration_screen = ("", "Calibration", "failed:", error)

    async def _calibrate_magnetometer(self):
        """Hard/soft iron correction (offset, scale) from the extremes seen while the device is rotated,
        None when it was not rotated enough"""
        low = [1e9, 1e9, 1e9]
        high = [-1e9, -1e9, -1e9]
        start = time.ticks_ms()
        while True:
            left = self._ROTATE_MS - time.ticks_diff(time.ticks_ms(), start)
            if left <= 0:
                break
            self.calibration_screen = ("1/2", "Rotate the", "device", f"{left // 1000 + 1} s")
            reading = self.mpu.magnetic
            # All zeros = no measurement ready yet
            if reading[0] or reading[1] or reading[2]:
                for i in range(3):
                    low[i] = min(low[i], reading[i])
                    high[i] = max(high[i], reading[i])
            await asyncio.sleep_ms(50)

        ranges = [high[i] - low[i] for i in range(3)]
        if min(ranges) < self._MAG_MIN_RANGE:
            return None
        avg_range = sum(ranges) / 3
        offset = tuple([(high[i] + low[i]) / 2 for i in range(3)])
        scale = tuple([avg_range / r for r in ranges])
        return offset, scale

    async def _calibrate_accelerometer(self):
        """Accelerometer bias from the device lying flat and still (gravity on Z only),
        None when it was not laid down in time"""
        start = time.ticks_ms()
        flat_since = None
        total = [0.0, 0.0, 0.0]
        count = 0
        while time.ticks_diff(time.ticks_ms(), start) < self._FLAT_TIMEOUT_MS:
            now = time.ticks_ms()
            reading = self.mpu.acceleration
            if abs(reading[0]) > self._FLAT_MAX_ACC or abs(reading[1]) > self._FLAT_MAX_ACC:
                # Not flat (yet, or moved): start over
                flat_since = None
                self.calibration_screen = ("2/2", "Lay down the", "device", "")
            else:
                if flat_since is None:
                    flat_since = now
                    total = [0.0, 0.0, 0.0]
                    count = 0
                for i in range(3):
                    total[i] += reading[i]
                count += 1
                if time.ticks_diff(now, flat_since) >= self._FLAT_HOLD_MS:
                    gravity = self._GRAVITY if total[2] >= 0 else -self._GRAVITY
                    return (total[0] / count, total[1] / count, total[2] / count - gravity)
                self.calibration_screen = ("2/2", "Lay down the", "device", "hold still")
            await asyncio.sleep_ms(50)
        return None

    def set_level(self):
        """Takes the current pitch and roll as zero (the menu, with the vehicle standing level).
        Returns the lines for the display: (step, line, line, line)"""
        if self.mpu is None:
            return "", "MPU not found", "", ""
        if not self.calibrated or self.calibrating:
            return "", "Calibrate the", "MPU first", ""
        old_level = self._level
        try:
            total = [0.0, 0.0, 0.0]
            for _ in range(self._LEVEL_SAMPLES):
                acc = self._mounted_acceleration()
                for i in range(3):
                    total[i] += acc[i]
            roll, pitch = self._tilt(*total)
            self._level = (degrees(pitch), degrees(roll))
            self._save_calibration()
        except OSError as e:
            print("Setting the level failed:", e)
            self._level = old_level
            return "", "Level", "failed:", "sensor error"
        print("MPU level saved:", self._level)
        return "", "Level", "saved", ""

    def _mounted_acceleration(self):
        acc = self.mpu.acceleration
        acc = (acc[0] - self._acc_bias[0], acc[1] - self._acc_bias[1], acc[2] - self._acc_bias[2])
        # The device is calibrated lying flat but mounted upright, turned 90 degrees around X:
        # sensor Y points up (where Z was) and sensor Z points where -Y was. Undo that rotation,
        # so the angles are the ones the device would give lying flat (0 pitch and roll when mounted)
        return acc[0], -acc[2], acc[1]

    @staticmethod
    def _tilt(ax, ay, az):
        """Roll and pitch in radians"""
        return atan2(ay, az), atan2(-ax, sqrt((ay ** 2) + (az ** 2)))

    def get_rotation(self):
        ax, ay, az = self._mounted_acceleration()
        raw_mx, raw_my, raw_mz = self.mpu.magnetic
        # Magnetometer axes expressed in the accelerometer frame
        mag = (raw_my, raw_mx, -raw_mz)
        # The same rotation to the mounted position as for the accelerometer
        mx, my, mz = mag[0], -mag[2], mag[1]

        roll, pitch = self._tilt(ax, ay, az)
        bx = (mx * cos(pitch)) + (sin(pitch) * ((my * sin(roll)) + (mz * cos(roll))))
        by = (mz * sin(roll)) - (my * cos(roll))
        yaw = atan2(-by, bx)
        # Pitch and roll are reported relative to the level set from the menu
        return degrees(pitch) - self._level[0], degrees(roll) - self._level[1], degrees(yaw)
