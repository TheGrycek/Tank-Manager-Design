import asyncio
import json
import os
import struct
import time
from math import log, pow, atan2, sqrt, degrees, cos, sin

import bluetooth
from machine import Pin, I2C, SoftI2C, ADC, WDT, reset, disable_irq, enable_irq

import aioble
import aioble.core
import aioble.device
import aioble.security
from mpu9250 import MPU9250
from sh1106 import SH1106_I2C

# Hardware watchdog timeout in ms, 0 = disabled.
# Once started it cannot be stopped (not even by Ctrl-C or a soft reset), so keep it at 0 while
# developing over the REPL and set it (e.g. 8000) for the installed device.
WATCHDOG_MS = 0

_IRQ_ENCRYPTION_UPDATE = 28
_IRQ_PASSKEY_ACTION = 31
_PASSKEY_ACTION_DISP = 3


class MPU:
    """MPU 9250 data calibration and retrieval"""
    _CALIBRATION_FILE = "mpu_calibration.json"

    def __init__(self):
        self._MPU_I2C_SCL_PIN = Pin(22)
        self._MPU_I2C_SDA_PIN = Pin(21)
        self._MPU_FREQ = 100000
        self.i2c = None
        self.mpu = None

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
            print("MPU calibration loaded")
        except (OSError, ValueError, KeyError):
            print("MPU calibration not found - yaw is uncalibrated, run mpu.calibrate_magnetometer()")

    def calibrate_magnetometer(self, count=256, delay=200):
        """Blocking (about a minute): run from the REPL and rotate the device around all axes."""
        offset, scale = self.mpu.ak8963.calibrate(count=count, delay=delay)
        with open(self._CALIBRATION_FILE, "w") as f:
            json.dump({"mag_offset": offset, "mag_scale": scale}, f)
        print("MPU calibration saved:", offset, scale)

    def get_rotation(self):
        ax, ay, az = self.mpu.acceleration
        raw_mx, raw_my, raw_mz = self.mpu.magnetic
        mx, my, mz = raw_my, raw_mx, -raw_mz

        roll = atan2(ay, az)
        pitch = atan2(-ax, sqrt((ay ** 2) + (az ** 2)))
        bx = (mx * cos(pitch)) + (sin(pitch) * ((my * sin(roll)) + (mz * cos(roll))))
        by = (mz * sin(roll)) - (my * cos(roll))
        yaw = atan2(-by, bx)
        return degrees(pitch), degrees(roll), degrees(yaw)


class OLED:
    """OLED display with SH1106 chip communication"""
    def __init__(self):
        self._OLED_I2C_SCL_PIN = Pin(18)
        self._OLED_I2C_SDA_PIN = Pin(19)
        self._OLED_WIDTH = 128
        self._OLED_HEIGHT = 64
        self._OLED_ADDR = 0x3C
        self.i2c = None
        self.oled = None
        self.valve_states = {
            0: "closed",
            1: "open",
            2: "closing",
            3: "opening"
        }

    def configure(self):
        try:
            self.i2c = SoftI2C(scl=self._OLED_I2C_SCL_PIN, sda=self._OLED_I2C_SDA_PIN)
            self.oled = SH1106_I2C(self._OLED_WIDTH, self._OLED_HEIGHT, self.i2c, addr=self._OLED_ADDR)
            self.oled.contrast(255)
            self.oled.fill(0)
            self.oled.text("System Ready", 0, 0)
            self.oled.show()
            return self.oled
        except OSError:
            print("Error: OLED not found")
            self.oled = None
            return None

    def print_tank_info(self, level_gray, level_clean, valve_state, temperature):
        if not self.oled: return
        temp = "T ERR" if temperature is None else f"{temperature:.1f}C"
        self.oled.fill(0)
        self.oled.text(f"|GRAY TANK>", 0, 0)
        self.oled.text(f" lvl:{level_gray:3d}% {temp}", 0, 15)
        self.oled.text(f" valve: {self.valve_states[valve_state]}", 0, 25)
        self.oled.text(f"|CLEAN TANK>", 0, 40)
        self.oled.text(f" lvl:{level_clean:3d}%", 0, 55)
        self.oled.show()

    def print_menu(self, position, count, name, value, editing):
        if not self.oled: return
        self.oled.fill(0)
        self.oled.text(f"|MENU {position}/{count}>", 0, 0)
        self.oled.text(f"{name}:", 0, 22)
        self.oled.text(f"> {value} <" if editing else f"{value}", 0, 40)
        self.oled.show()

    def print_passkey(self, passkey):
        if not self.oled: return
        self.oled.fill(0)
        self.oled.text("|BLUETOOTH>", 0, 0)
        self.oled.text(" pairing code:", 0, 20)
        self.oled.text(f"   {passkey:06d}", 0, 40)
        self.oled.show()


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
        super().__init__(32, r_empty=0.0, r_full=190.0)


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
        self._TEMP_SENSOR_PIN = Pin(34, Pin.IN)
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


class Valve:
    """Electric valve regulation"""
    MOVE_TIME_SEC = 35  # valve movement time (about 30 s, its limit switch cuts the motor)
    AUTO_CLOSE_SEC = 600  # auto-close time

    def __init__(self):
        self._OPEN_PIN = Pin(2, Pin.OUT)
        self._CLOSE_PIN = Pin(16, Pin.OUT)

        self._STATE = 0 # 0-closed, 1-open, 2-closing, 3-opening
        self._move_task = None
        self._auto_close_task = None

    def configure(self):
        self.stop()

    def stop(self):
        self._OPEN_PIN.value(0)
        self._CLOSE_PIN.value(0)

    def get_state(self):
        return self._STATE

    def get_target(self):
        # What the app shows: 1 when open or opening, 0 when closed or closing
        return 1 if self._STATE in (1, 3) else 0

    async def _run_valve(self, state):
        try:
            # Ensure both pins are off first
            self.stop()

            if state == 1:
                self._OPEN_PIN.value(1)
                self._STATE = 3
                print("Valve OPENING")
                await asyncio.sleep(self.MOVE_TIME_SEC)
                self._STATE = 1
            else:
                self._CLOSE_PIN.value(1)
                self._STATE = 2
                print("Valve CLOSING")
                await asyncio.sleep(self.MOVE_TIME_SEC)
                self._STATE = 0

        except asyncio.CancelledError:
            print("Valve movement cancelled")
            raise
        finally:
            # Always stop power to motor
            self.stop()
            print("Valve motor stopped")

    async def _auto_close(self, ble):
        try:
            await asyncio.sleep(self.AUTO_CLOSE_SEC)
        except asyncio.CancelledError:
            print("Auto-close cancelled")
            raise

        # This task is done waiting: forget it, so starting the movement does not cancel itself
        self._auto_close_task = None
        if self._STATE in (1, 3):
            print("Auto-closing valve after timeout")
            await self._start_valve_task(0)
            await ble.write_characteristic("VALVE", 0, '<B')

    async def _start_valve_task(self, state, ble=None):
        # Cancel active movement
        if self._move_task is not None:
            self._move_task.cancel()
            try:
                await self._move_task
            except asyncio.CancelledError:
                pass

        # Cancel pending auto-close
        if self._auto_close_task is not None:
            self._auto_close_task.cancel()
            self._auto_close_task = None

        # Start valve movement
        self._move_task = asyncio.create_task(self._run_valve(state))

        # If opening, schedule auto-close
        if state == 1 and ble is not None:
            self._auto_close_task = asyncio.create_task(self._auto_close(ble))

    async def request(self, state, ble):
        """Open (1) or close (0) the valve; used by the app command and by the panel menu"""
        s = self.get_state()
        if (state == 0 and s in (1, 3)) or (state == 1 and s in (0, 2)):
            print(f"Valve movement requested: {state}")
            await self._start_valve_task(state, ble)

        # Confirms the accepted target to the app
        await ble.write_characteristic("VALVE", state, '<B')

    async def control(self, ble):
        while True:
            try:
                try:
                    connection, data = await ble.characteristics["VALVE"].written(timeout_ms=200)
                except asyncio.TimeoutError:
                    # Nothing written by the app
                    continue

                command = ble.decode_data(data)
                if command not in (0, 1):
                    print(f"Ignoring invalid valve command: {command}")
                    continue

                print(f"App command received: {command}")
                await self.request(command, ble)

            except asyncio.CancelledError:
                print("Valve control task cancelled")

                # Stop any running movement
                if self._move_task is not None:
                    self._move_task.cancel()

                # Cancel auto close task
                if self._auto_close_task is not None:
                    self._auto_close_task.cancel()

                self.stop()
                break

            except Exception as e:
                print("Error in valve control task:", e)
                await asyncio.sleep_ms(200)


class Heater:
    def __init__(self):
        self._HEATER_PIN = Pin(4, Pin.OUT)
        self._STATE = 0
        self._HYSTERESIS = 1.0  # heater turns off this many degrees above the threshold
        self.sensor_fault = False

    def configure(self):
        self.control_heater(0)

    def control_heater(self, state):
        self._STATE = state
        self._HEATER_PIN.value(state)

    def get_state(self):
        return self._STATE

    async def control(self, sensor, ble, settings):
        while True:
            try:
                temperature = sensor.get_temperature()
                current_state = self.get_state()

                new_state = current_state
                if not settings.auto_heating:
                    new_state = 0
                elif temperature is None:
                    # Broken sensor: never heat blindly
                    if not self.sensor_fault:
                        print("Temperature sensor fault - heater disabled")
                    self.sensor_fault = True
                    new_state = 0
                else:
                    self.sensor_fault = False
                    if temperature <= settings.heating_threshold:
                        new_state = 1 # Turn ON
                    elif temperature >= (settings.heating_threshold + self._HYSTERESIS):
                        new_state = 0 # Turn OFF

                if new_state != current_state:
                    self.control_heater(new_state)
                    print(f"Heater state changed to: {new_state}")
                    await ble.write_characteristic("HEATER", new_state, '<B')

                await asyncio.sleep(1)

            except asyncio.CancelledError:
                print("Heater task cancelled")
                self.control_heater(0)
                break
            except Exception as e:
                print("Error in heater control task:", e)
                self.control_heater(0)
                await asyncio.sleep(5)


class Buzzer:
    """Gray tank overfill alarm"""
    def __init__(self):
        self._BUZZER_PIN = Pin(13, Pin.OUT)

    def configure(self):
        self.off()

    def off(self):
        self._BUZZER_PIN.value(0)

    def control(self, valve_state, gray_lvl, settings):
        # Sound while the gray tank is above the threshold and the valve is still closed
        alarm = settings.buzzer_on and valve_state == 0 and gray_lvl > settings.gray_lvl_threshold
        self._BUZZER_PIN.value(1 if alarm else 0)


class Settings:
    """User settings, changed from the panel menu or from the app, kept in flash"""
    _FILE = "settings.json"
    GRAY_LVL_MIN = 50
    GRAY_LVL_MAX = 100
    GRAY_LVL_STEP = 5
    HEATING_MIN = 1
    HEATING_MAX = 5

    def __init__(self):
        self.buzzer_on = True
        self.gray_lvl_threshold = 90   # %
        self.auto_heating = True
        self.heating_threshold = 4     # Celsius

    def load(self):
        try:
            with open(self._FILE) as f:
                data = json.load(f)
            self.update(data.get("buzzer_on"), data.get("gray_lvl_threshold"),
                        data.get("auto_heating"), data.get("heating_threshold"), save=False)
            print("Settings loaded")
        except (OSError, ValueError):
            print("Settings not found - using defaults")

    def save(self):
        try:
            with open(self._FILE, "w") as f:
                json.dump({
                    "buzzer_on": self.buzzer_on,
                    "gray_lvl_threshold": self.gray_lvl_threshold,
                    "auto_heating": self.auto_heating,
                    "heating_threshold": self.heating_threshold,
                }, f)
        except OSError as e:
            print("Saving settings failed:", e)

    def update(self, buzzer_on=None, gray_lvl_threshold=None, auto_heating=None, heating_threshold=None, save=True):
        """Single place where values are validated; out-of-range values are clamped. True if changed."""
        before = self.pack()
        if buzzer_on is not None:
            self.buzzer_on = bool(buzzer_on)
        if gray_lvl_threshold is not None:
            self.gray_lvl_threshold = min(max(int(gray_lvl_threshold), self.GRAY_LVL_MIN), self.GRAY_LVL_MAX)
        if auto_heating is not None:
            self.auto_heating = bool(auto_heating)
        if heating_threshold is not None:
            self.heating_threshold = min(max(int(heating_threshold), self.HEATING_MIN), self.HEATING_MAX)
        changed = self.pack() != before
        if changed and save:
            self.save()
        return changed

    def pack(self):
        # BLE payload of the SETTINGS characteristic
        return struct.pack('<4B', 1 if self.buzzer_on else 0, self.gray_lvl_threshold,
                           1 if self.auto_heating else 0, self.heating_threshold)


class Encoder:
    """EC12 rotary encoder with push button.
    Common pin and one switch pin go to GND; A, B and SW use the internal pull-ups (active low)."""
    _TRANSITIONS_PER_STEP = 4  # one detent = one full quadrature cycle
    # Index: (previous AB << 2) | current AB; value: direction of a valid transition
    _TABLE = (0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0)
    _DEBOUNCE_POLLS = 2

    def __init__(self):
        self._A = Pin(17, Pin.IN, Pin.PULL_UP)
        self._B = Pin(23, Pin.IN, Pin.PULL_UP)
        self._SW = Pin(5, Pin.IN, Pin.PULL_UP)
        self._last = 0
        self._transitions = 0
        self._pressed = False
        self._stable_polls = 0

    def configure(self):
        self._last = (self._A.value() << 1) | self._B.value()
        self._A.irq(self._on_edge, Pin.IRQ_RISING | Pin.IRQ_FALLING)
        self._B.irq(self._on_edge, Pin.IRQ_RISING | Pin.IRQ_FALLING)

    def _on_edge(self, pin):
        current = (self._A.value() << 1) | self._B.value()
        self._transitions += self._TABLE[(self._last << 2) | current]
        self._last = current

    def take_steps(self):
        """Detents turned since the last call (positive = clockwise)"""
        state = disable_irq()
        steps = int(self._transitions / self._TRANSITIONS_PER_STEP)
        self._transitions -= steps * self._TRANSITIONS_PER_STEP
        enable_irq(state)
        return steps

    def poll_button(self):
        """Call periodically; True once per (debounced) press"""
        raw = self._SW.value() == 0
        if raw == self._pressed:
            self._stable_polls = 0
            return False
        self._stable_polls += 1
        if self._stable_polls < self._DEBOUNCE_POLLS:
            return False
        self._stable_polls = 0
        self._pressed = raw
        return raw


class Menu:
    """Settings menu on the display, driven by the encoder"""
    IDLE = 0     # normal tank screen
    BROWSE = 1   # turning selects the item
    EDIT = 2     # turning changes the value of the selected item
    _TIMEOUT_MS = 15000

    BUZZER = 0
    GRAY_LVL_THR = 1
    AUTO_HEATING = 2
    HEATING_THR = 3
    VALVE = 4
    _NAMES = ("BUZZER", "GRAY LVL THR", "AUTO-HEATING", "HEATING THR", "VALVE")

    def __init__(self, settings, valve, ble):
        self._settings = settings
        self._valve = valve
        self._ble = ble
        self.state = self.IDLE
        self.item = 0
        self._edit_value = 0
        self._last_activity = 0

    def _value_text(self):
        s = self._settings
        if self.item == self.BUZZER:
            return "ON" if s.buzzer_on else "OFF"
        if self.item == self.GRAY_LVL_THR:
            return f"{self._edit_value if self.state == self.EDIT else s.gray_lvl_threshold}%"
        if self.item == self.AUTO_HEATING:
            return "ON" if s.auto_heating else "OFF"
        if self.item == self.HEATING_THR:
            return f"{self._edit_value if self.state == self.EDIT else s.heating_threshold}'C"
        return "OPEN" if self._valve.get_target() else "CLOSED"

    def screen(self):
        """What to draw: (position, count, name, value, editing)"""
        return (self.item + 1, len(self._NAMES), self._NAMES[self.item], self._value_text(), self.state == self.EDIT)

    async def _commit_edit(self):
        if self.item == self.GRAY_LVL_THR:
            changed = self._settings.update(gray_lvl_threshold=self._edit_value)
        else:
            changed = self._settings.update(heating_threshold=self._edit_value)
        if changed:
            await self._ble.send_settings()

    async def handle(self, steps, pressed):
        """Feed encoder input; returns True when the display needs a redraw"""
        now = time.ticks_ms()
        if not steps and not pressed:
            if self.state != self.IDLE and time.ticks_diff(now, self._last_activity) > self._TIMEOUT_MS:
                if self.state == self.EDIT:
                    await self._commit_edit()
                self.state = self.IDLE
                return True
            return False
        self._last_activity = now
        s = self._settings

        if self.state == self.IDLE:
            # Any press opens the menu; turning alone does nothing on the tank screen
            if pressed:
                self.state = self.BROWSE
                return True
            return False

        if self.state == self.BROWSE:
            if steps:
                self.item = (self.item + steps) % len(self._NAMES)
            if pressed:
                if self.item == self.BUZZER:
                    s.update(buzzer_on=not s.buzzer_on)
                    await self._ble.send_settings()
                elif self.item == self.AUTO_HEATING:
                    s.update(auto_heating=not s.auto_heating)
                    await self._ble.send_settings()
                elif self.item == self.VALVE:
                    await self._valve.request(0 if self._valve.get_target() else 1, self._ble)
                else:
                    self._edit_value = s.gray_lvl_threshold if self.item == self.GRAY_LVL_THR else s.heating_threshold
                    self.state = self.EDIT
            return True

        # EDIT
        if steps:
            if self.item == self.GRAY_LVL_THR:
                value = self._edit_value + steps * s.GRAY_LVL_STEP
                self._edit_value = min(max(value, s.GRAY_LVL_MIN), s.GRAY_LVL_MAX)
            else:
                self._edit_value = min(max(self._edit_value + steps, s.HEATING_MIN), s.HEATING_MAX)
        if pressed:
            await self._commit_edit()
            self.state = self.BROWSE
        return True


class Bluetooth:
    # Characteristics the app must always see up to date: indicated on change, re-sent periodically
    _STATE_CHARACTERISTICS = ("VALVE", "VALVE_LOCKED", "HEATER")

    def __init__(self):
        self._ADV_INTERVAL_US = 250000
        self._PAIR_TIMEOUT_MS = 60000  # time for the user to type the code shown on the display
        self._BT_NAME = "Mati&Mati"
        self._MANAGER_SERVICE_UUID = bluetooth.UUID('19b10000-e8f2-537e-4f6c-d104768a1214')
        self.characteristic_uuids = {
            "CLEAN_LEVEL_SENSOR": bluetooth.UUID('00000000-0000-0000-0000-000000000001'),
            "GRAY_LEVEL_SENSOR": bluetooth.UUID('00000000-0000-0000-0000-000000000002'),
            "GRAY_TEMP_SENSOR": bluetooth.UUID('00000000-0000-0000-0000-000000000003'),
            "MPU_SENSOR": bluetooth.UUID('00000000-0000-0000-0000-000000000004'),
            "VALVE": bluetooth.UUID('00000000-0000-0000-0000-000000000005'),
            "VALVE_LOCKED": bluetooth.UUID('00000000-0000-0000-0000-000000000006'),
            "HEATER": bluetooth.UUID('00000000-0000-0000-0000-000000000007'),
            "SETTINGS": bluetooth.UUID('00000000-0000-0000-0000-000000000008'),
        }
        self.characteristics = {}
        # Pairing code currently shown on the display, None when no pairing is in progress
        self.passkey = None

    def configure(self):
        aioble.config(gap_name=self._BT_NAME, bond=True, mitm=True, le_secure=True, io=0)
        ble_service = aioble.Service(self._MANAGER_SERVICE_UUID)

        _FLAG_READ_AUTHENTICATED = 0x0400
        _FLAG_WRITE_AUTHENTICATED = 0x2000

        for name in self.characteristic_uuids:
            if name == "VALVE":
                self.characteristics[name] = aioble.Characteristic(
                    ble_service, self.characteristic_uuids[name], read=True, write=True, capture=True,
                    notify=True, indicate=True
                )
                self.characteristics[name].flags |= (_FLAG_READ_AUTHENTICATED | _FLAG_WRITE_AUTHENTICATED)
            elif name == "SETTINGS":
                self.characteristics[name] = aioble.Characteristic(
                    ble_service, self.characteristic_uuids[name], read=True, write=True, capture=True,
                    notify=True
                )
                self.characteristics[name].flags |= (_FLAG_READ_AUTHENTICATED | _FLAG_WRITE_AUTHENTICATED)
            elif name in self._STATE_CHARACTERISTICS:
                self.characteristics[name] = aioble.Characteristic(
                    ble_service, self.characteristic_uuids[name], read=True, notify=True, indicate=True
                )
                self.characteristics[name].flags |= _FLAG_READ_AUTHENTICATED
            else:
                self.characteristics[name] = aioble.Characteristic(
                    ble_service, self.characteristic_uuids[name], read=True, notify=True
                )
                self.characteristics[name].flags |= _FLAG_READ_AUTHENTICATED

        aioble.register_services(ble_service)
        aioble.core.register_irq_handler(self._security_handler, None)
        print("BLE Service Registered and Advertising")

    def _security_handler(self, event, data):
        if event == _IRQ_PASSKEY_ACTION:
            conn_handle, action, passkey = data
            if action == _PASSKEY_ACTION_DISP:
                # A fresh random 6-digit code for every pairing, shown on the display
                self.passkey = int.from_bytes(os.urandom(4), 'little') % 1000000
                print(f"Pairing code: {self.passkey:06d}")
                bluetooth.BLE().gap_passkey(conn_handle, action, self.passkey)
                return True
        elif event == _IRQ_ENCRYPTION_UPDATE:
            conn_handle, encrypted, authenticated, bonded, key_size = data
            # Pairing is over, successful or not
            self.passkey = None
            if not encrypted:
                # aioble only wakes pair() on success: wake it on failure too, so that a phone
                # with a stale bond is dropped at once instead of blocking for the whole timeout
                connection = aioble.device.DeviceConnection._connected.get(conn_handle, None)
                if connection and connection._pair_event:
                    connection._pair_event.set()
        return None

    @staticmethod
    def decode_data(data):
        try:
            if data is not None and len(data) > 0:
                return int.from_bytes(data, 'little')
        except Exception as e:
            print(f"Error decoding data: {data}", e)
        return None

    async def write_characteristic(self, name, data, fmt, indicate=True):
        await self._send(name, struct.pack(fmt, data), indicate)

    async def send_settings(self):
        await self._send("SETTINGS", settings.pack(), False)

    async def _send(self, name, encoded_data, indicate):
        # Update local value but do not broadcast to all
        self.characteristics[name].write(encoded_data, send_update=False)

        # Manually notify/indicate only authenticated connections
        for conn in list(aioble.device.DeviceConnection._connected.values()):
            if conn.authenticated:
                if indicate and name in self._STATE_CHARACTERISTICS:
                    try:
                        await self.characteristics[name].indicate(conn, encoded_data)
                    except Exception as e:
                        print(f"Indicate failed for {name}: {e}")
                else:
                    self.characteristics[name].notify(conn, encoded_data)

    async def write_characteristic_multiple(self, name, data_tuple, fmt):
        encoded_data = struct.pack(fmt, *data_tuple)
        # Update local value but do not broadcast to all
        self.characteristics[name].write(encoded_data, send_update=False)

        # Manually notify only authenticated connections
        for conn in list(aioble.device.DeviceConnection._connected.values()):
            if conn.authenticated:
                self.characteristics[name].notify(conn, encoded_data)

    async def refresh_state(self):
        # Cheap periodic re-send (no acknowledgement) so that an app which subscribed late,
        # e.g. after a reconnect, still gets the current state
        await self.write_characteristic("VALVE", valve.get_target(), '<B', indicate=False)
        # VALVE_LOCKED is kept for the app protocol; there is no physical valve button any more
        await self.write_characteristic("VALVE_LOCKED", 0, '<B', indicate=False)
        await self.write_characteristic("HEATER", heater.get_state(), '<B', indicate=False)
        await self.send_settings()

    async def advertise(self):
        print("Starting BLE advertising...")
        while True:
            try:
                async with await aioble.advertise(
                        self._ADV_INTERVAL_US,
                        name=self._BT_NAME,
                        services=[self._MANAGER_SERVICE_UUID]
                ) as connection:
                    print("Connection from", connection.device)

                    if not connection.authenticated:
                        try:
                            print("Requesting pairing...")
                            await connection.pair(bond=True, mitm=True, le_secure=True, io=0,
                                                  timeout_ms=self._PAIR_TIMEOUT_MS)
                            if not connection.authenticated:
                                raise OSError("link is not authenticated")
                            print("Pairing complete")
                        except Exception as e:
                            print(f"Pairing failed: {e}")
                            await connection.disconnect()
                            continue
                        finally:
                            self.passkey = None

                    await self.write_characteristic("VALVE", valve.get_target(), '<B')
                    await self.write_characteristic("VALVE_LOCKED", 0, '<B')
                    await self.write_characteristic("HEATER", heater.get_state(), '<B')
                    await self.send_settings()
                    await connection.disconnected()
                    print("Disconnected from", connection.device)
            except asyncio.CancelledError:
                print("Advertise task cancelled")
                break
            except Exception as e:
                print("Error in advertise task:", e)
            finally:
                self.passkey = None
                await asyncio.sleep_ms(100)


ble = Bluetooth()
mpu = MPU()
oled = OLED()
gray_lvl_sensor = GrayLevelSensor()
clean_lvl_sensor = CleanLevelSensor()
temp_sensor = TemperatureSensor()
heater = Heater()
valve = Valve()
buzzer = Buzzer()
settings = Settings()
encoder = Encoder()
menu = Menu(settings, valve, ble)
# Latest sensor readings, shared between the sensor task and the display task
readings = {"gray": 0, "clean": 0, "temp": None}

async def mpu_sensor_task():
    while True:
        try:
            if mpu.mpu is None and not mpu.configure():
                # Sensor missing: keep trying, it may only be a loose ribbon
                await asyncio.sleep(10)
                continue
            pitch, roll, yaw = mpu.get_rotation()
            await ble.write_characteristic_multiple(
                "MPU_SENSOR", (pitch, roll, yaw), '<3f')
        except Exception as e:
            print(f"Error in MPU task: {e}")
            mpu.mpu = None
        await asyncio.sleep(1)

async def environmental_sensors_task():
    while not (gray_lvl_sensor.configure() and clean_lvl_sensor.configure() and temp_sensor.configure()):
        print("One or more environmental sensors failed to configure, retrying.")
        await asyncio.sleep(10)
    cycle = 0
    while True:
        try:
            gray_lvl = gray_lvl_sensor.get_level()
            clean_lvl = clean_lvl_sensor.get_level()
            temp_celsius = temp_sensor.get_temperature()
            temp_raw = TemperatureSensor.INVALID if temp_celsius is None else int(temp_celsius * 100)

            await ble.write_characteristic("GRAY_LEVEL_SENSOR", gray_lvl, '<B')
            await ble.write_characteristic("CLEAN_LEVEL_SENSOR", clean_lvl, '<B')
            await ble.write_characteristic("GRAY_TEMP_SENSOR", temp_raw, '<h')
            if cycle % 2 == 0:
                await ble.refresh_state()
            cycle += 1

            readings["gray"] = gray_lvl
            readings["clean"] = clean_lvl
            readings["temp"] = temp_celsius
            buzzer.control(valve.get_state(), gray_lvl, settings)

        except Exception as e:
            print(f"Error in environmental sensors task: {e}")
        await asyncio.sleep(1)

async def settings_task():
    # Settings written by the app
    characteristic = ble.characteristics["SETTINGS"]
    while True:
        try:
            try:
                connection, data = await characteristic.written(timeout_ms=500)
            except asyncio.TimeoutError:
                continue
            if data is None or len(data) < 4:
                print(f"Ignoring invalid settings: {data}")
            elif settings.update(data[0], data[1], data[2], data[3]):
                print("Settings changed from the app")
            # Always answer with the values actually in force (they may have been clamped)
            await ble.send_settings()
        except asyncio.CancelledError:
            break
        except Exception as e:
            print(f"Error in settings task: {e}")
            await asyncio.sleep_ms(200)

async def ui_task():
    # The only task that draws on the display: pairing code > menu > tank screen
    encoder.configure()
    last_draw = None
    last_refresh = time.ticks_ms()
    while True:
        try:
            redraw = await menu.handle(encoder.take_steps(), encoder.poll_button())
            now = time.ticks_ms()
            # Values on screen (levels, valve state) also change on their own
            if time.ticks_diff(now, last_refresh) >= 500:
                redraw = True
            if redraw:
                last_refresh = now
                if ble.passkey is not None:
                    draw = ("passkey", ble.passkey)
                elif menu.state != Menu.IDLE:
                    draw = ("menu",) + menu.screen()
                else:
                    draw = ("tank", readings["gray"], readings["clean"], valve.get_state(), readings["temp"])
                if draw != last_draw:
                    last_draw = draw
                    if draw[0] == "passkey":
                        oled.print_passkey(draw[1])
                    elif draw[0] == "menu":
                        oled.print_menu(*draw[1:])
                    else:
                        oled.print_tank_info(*draw[1:])
        except asyncio.CancelledError:
            break
        except Exception as e:
            print(f"Error in UI task: {e}")
        await asyncio.sleep_ms(50)

async def watchdog_task():
    wdt = WDT(timeout=WATCHDOG_MS)
    while True:
        wdt.feed()
        await asyncio.sleep_ms(WATCHDOG_MS // 4)

def safe_outputs():
    # Heater and valve motor off, whatever state the program stopped in
    heater.control_heater(0)
    valve.stop()
    buzzer.off()

async def main():
    ble.configure()
    oled.configure()
    valve.configure()
    heater.configure()
    buzzer.configure()
    settings.load()

    tasks = [
        asyncio.create_task(ble.advertise()),
        asyncio.create_task(environmental_sensors_task()),
        asyncio.create_task(mpu_sensor_task()),
        asyncio.create_task(heater.control(temp_sensor, ble, settings)),
        asyncio.create_task(valve.control(ble)),
        asyncio.create_task(settings_task()),
        asyncio.create_task(ui_task()),
    ]
    if WATCHDOG_MS:
        tasks.append(asyncio.create_task(watchdog_task()))
    await asyncio.gather(*tasks)

restart = True
try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("Program stopped by user")
    restart = False
except Exception as e:
    print(f"An unexpected error occurred in main run: {e}")
finally:
    safe_outputs()
    aioble.stop()
    print("BLE stopped")

if restart:
    # The controller must not stay dead after a crash
    print("Restarting...")
    reset()
