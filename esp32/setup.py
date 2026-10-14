import asyncio
import json
import os
import struct
import time

import bluetooth
from micropython import schedule

import aioble
import aioble.core
import aioble.device
import aioble.security

_IRQ_ENCRYPTION_UPDATE = 28
_IRQ_GET_SECRET = 29
_IRQ_SET_SECRET = 30
_IRQ_PASSKEY_ACTION = 31
_PASSKEY_ACTION_DISP = 3
# Bond records (NimBLE ble_store_value_sec) kept by aioble.security, keyed by (type, peer address)
_SECRET_OUR_SEC = 1
_SECRET_PEER_SEC = 2
_PEER_SEC_IRK = 41  # offset of the peer's IRK (16 bytes, little-endian), followed by irk_present


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


class Menu:
    """Settings menu on the display, driven by the encoder"""
    IDLE = 0     # normal tank screen
    BROWSE = 1   # turning selects the item
    EDIT = 2     # turning changes the value of the selected item
    CALIBRATE = 3  # MPU calibration in progress, the display shows what to do with the device
    _TIMEOUT_MS = 15000
    _RESULT_MS = 5000  # how long the calibration result stays on the display

    BUZZER = 0
    GRAY_LVL_THR = 1
    AUTO_HEATING = 2
    HEATING_THR = 3
    VALVE = 4
    MPU = 5
    LEVEL = 6
    _NAMES = ("BUZZER", "GRAY LVL THR", "AUTO-HEATING", "HEATING THR", "VALVE", "MPU", "LEVEL")

    def __init__(self, settings, valve, ble, mpu):
        self._settings = settings
        self._valve = valve
        self._ble = ble
        self._mpu = mpu
        self._calibration_task = None
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
        if self.item == self.MPU:
            return "CALIBRATED" if self._mpu.calibrated else "NOT CALIBRATED"
        if self.item == self.LEVEL:
            return "SET ZERO"
        return "OPEN" if self._valve.get_target() else "CLOSED"

    def screen(self):
        """What to draw: (position, count, name, value, editing)"""
        return (self.item + 1, len(self._NAMES), self._NAMES[self.item], self._value_text(), self.state == self.EDIT)

    def calibration_screen(self):
        """What to draw in the CALIBRATE state: (step, line, line, line)"""
        return self._mpu.calibration_screen

    def _handle_calibration(self, pressed, now):
        task = self._calibration_task
        if task is not None and task.done():
            # Finished: the result stays on the display for a moment
            self._calibration_task = task = None
            self._last_activity = now
        if pressed:
            # A press cancels a running calibration or dismisses its result
            if task is not None:
                task.cancel()
                self._calibration_task = None
        elif task is not None or time.ticks_diff(now, self._last_activity) < self._RESULT_MS:
            return False
        self.state = self.BROWSE
        self._last_activity = now
        return True

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
        if self.state == self.CALIBRATE:
            # Turning does nothing and the menu timeout does not apply here
            return self._handle_calibration(pressed, now)
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
                elif self.item == self.MPU:
                    self._calibration_task = self._mpu.start_calibration()
                    self.state = self.CALIBRATE
                elif self.item == self.LEVEL:
                    # Shown like a calibration result
                    self._mpu.calibration_screen = self._mpu.set_level()
                    self.state = self.CALIBRATE
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
    _CHARACTERISTIC_ORDER = ("CLEAN_LEVEL_SENSOR", "GRAY_LEVEL_SENSOR", "GRAY_TEMP_SENSOR", "MPU_SENSOR",
                             "VALVE", "VALVE_LOCKED", "HEATER", "SETTINGS")

    def __init__(self, settings, valve, heater):
        self._settings = settings
        self._valve = valve
        self._heater = heater
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

        # Registered in a fixed order (MicroPython dicts do not keep insertion order), so the
        # attribute handles stay the same; new characteristics go at the end
        for name in self._CHARACTERISTIC_ORDER:
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
        # Ahead of aioble.security's handler, which must not see bond lookups by a private address
        aioble.core._irq_handlers.insert(0, self._security_handler)
        print("BLE Service Registered and Advertising")

    @staticmethod
    def _bonded_key(key):
        """Stored key (type + identity address) of the bonded phone behind a resolvable private
        address, or None.

        Phones connect from a random address that changes with every connection. The ESP32
        controller does not resolve it in MicroPython, so NimBLE would look the bond up under that
        address, not find it and ask for pairing again. Resolve it here with the IRK the phone gave
        during pairing (Bluetooth Core Spec, vol 3 part H, 2.2.2: hash = AES(IRK, prand) mod 2^24)."""
        if key is None or len(key) != 7 or key[0] != 1:  # random address only
            return None
        address = bytes(reversed(key[1:]))  # most significant byte first
        if address[0] >> 6 != 0b01:  # resolvable private address only
            return None
        # Imported here, not at start-up: importing it before Bluetooth is up breaks the BLE init
        import cryptolib
        plaintext = bytes(13) + address[:3]
        for (sec_type, stored_key), value in aioble.security._secrets.items():
            if sec_type != _SECRET_PEER_SEC or len(value) <= _PEER_SEC_IRK + 16 or not value[_PEER_SEC_IRK + 16]:
                continue
            irk = bytes(reversed(value[_PEER_SEC_IRK:_PEER_SEC_IRK + 16]))
            if cryptolib.aes(irk, 1).encrypt(plaintext)[13:] == address[3:]:
                return stored_key
        return None

    def _bond_secret(self, event, data):
        """Bond reads and writes by a private address go to the record of the phone's identity"""
        if event == _IRQ_GET_SECRET:
            sec_type, index, key = data
        else:
            sec_type, key, value = data
        if sec_type not in (_SECRET_OUR_SEC, _SECRET_PEER_SEC):
            return None
        identity = self._bonded_key(bytes(key) if key else None)
        if identity is None:
            return None  # aioble.security handles it as usual
        secrets = aioble.security._secrets
        if event == _IRQ_GET_SECRET:
            return secrets.get((sec_type, identity), None)
        if value:
            secrets[sec_type, identity] = bytes(value)
        elif (sec_type, identity) in secrets:
            # The phone has dropped the bond and pairs again
            del secrets[sec_type, identity]
        aioble.security._modified = True
        schedule(aioble.security._save_secrets, None)
        return True

    def _security_handler(self, event, data):
        if event == _IRQ_GET_SECRET or event == _IRQ_SET_SECRET:
            return self._bond_secret(event, data)
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
        await self._send("SETTINGS", self._settings.pack(), False)

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
        await self.write_characteristic("VALVE", self._valve.get_target(), '<B', indicate=False)
        # VALVE_LOCKED is kept for the app protocol; there is no physical valve button any more
        await self.write_characteristic("VALVE_LOCKED", 0, '<B', indicate=False)
        await self.write_characteristic("HEATER", self._heater.get_state(), '<B', indicate=False)
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

                    await self.write_characteristic("VALVE", self._valve.get_target(), '<B')
                    await self.write_characteristic("VALVE_LOCKED", 0, '<B')
                    await self.write_characteristic("HEATER", self._heater.get_state(), '<B')
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