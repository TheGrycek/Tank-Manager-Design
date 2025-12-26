import asyncio
import struct
from math import log, pow, atan2, sqrt, degrees, cos, sin

import bluetooth
from machine import Pin, I2C, SoftI2C, ADC

import aioble
from mpu9250 import MPU9250
from sh1106 import SH1106_I2C

PASSKEY = 123456


class MPU:
    """MPU 9250 data calibration and retrieval"""
    def __init__(self):
        self._ACC_BIAS = [-77.756095, 240.097564, 970.439453]
        self._GYRO_BIAS = [94.780487, 119.024391, -12.048781]
        self._MAG_BIAS = [160.030548, 406.369689, -424.748931]
        self._MAG_SCALE = [0.957207, 1.045511, 1.001178]

        self._MPU_I2C_SCL_PIN = Pin(22)
        self._MPU_I2C_SDA_PIN = Pin(21)
        self._MPU_FREQ = 100000
        self.i2c = None
        self.mpu = None

    def configure(self):
        try:
            self.i2c = I2C(
                scl=self._MPU_I2C_SCL_PIN,
                sda=self._MPU_I2C_SDA_PIN,
                freq=self._MPU_FREQ
            )
            self.mpu = MPU9250(self.i2c)
            print("MPU9250 id: " + hex(self.mpu.whoami))
            return self.mpu
        except OSError:
            print("Error: MPU not found")
            return None

    def get_scaled_data(self):
        acc = [self.mpu.acceleration[i] - self._ACC_BIAS[i] for i in range(3)]
        gyro = [self.mpu.gyro[i] - self._GYRO_BIAS[i] for i in range(3)]
        mag = [(self.mpu.magnetic[i] - self._MAG_BIAS[i]) * self._MAG_SCALE[i] for i in range(3)]
        return acc, gyro, mag

    def get_rotation(self):
        acc, gyro, mag = self.get_scaled_data()
        roll = atan2(acc[1], acc[2])
        pitch = atan2(-acc[0], sqrt((acc[1] ** 2) + (acc[2] ** 2)))
        bx = (mag[0] * cos(pitch)) + (sin(pitch) * ((mag[1] * sin(roll)) + (mag[2] * cos(roll))))
        by = (mag[2] * sin(roll)) - (mag[1] * cos(roll))
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
            return None

    def print_tank_info(self, level_gray, level_clean, valve):
        if not self.oled: return
        self.oled.fill(0)
        self.oled.text(f"|GRAY TANK>", 0, 0)
        self.oled.text(f" lvl: {level_gray:.1f}% ", 0, 15)
        self.oled.text(f" valve: {'open' if valve else 'closed'}", 0, 25)
        self.oled.text(f"|CLEAN TANK>", 0, 40)
        self.oled.text(f" lvl: {level_clean:.1f}%", 0, 55)
        self.oled.show()

    def print_rotation_info(self, pitch, roll, yaw):
        if not self.oled: return
        self.oled.fill(0)
        self.oled.text(f"|ROTATION>", 0, 0)
        self.oled.text(f" pitch: {pitch:.1f}", 0, 15)
        self.oled.text(f" roll: {roll:.1f}", 0, 30)
        self.oled.text(f" yaw: {yaw:.1f}", 0, 45)
        self.oled.show()

class GrayLevelSensor:
    """Gray water tank level sensor"""
    def __init__(self):
        self._LVL_SENSOR_PIN = Pin(32, Pin.IN)
        self._MAX_ADC = 4095
        self._MAX_LVL = 100
        self.adc = None

    def configure(self):
        try:
            self.adc = ADC(self._LVL_SENSOR_PIN)
            self.adc.atten(ADC.ATTN_11DB)
            return self.adc
        except OSError: return None

    def get_level(self):
        return int((self.adc.read() / self._MAX_ADC) * self._MAX_LVL)

class CleanLevelSensor:
    """Clean water tank level sensor"""
    def __init__(self):
        self._LVL_SENSOR_PIN = Pin(35, Pin.IN)
        self._MAX_ADC = 4095
        self._MAX_LVL = 100
        self.adc = None

    def configure(self):
        try:
            self.adc = ADC(self._LVL_SENSOR_PIN)
            self.adc.atten(ADC.ATTN_11DB)
            return self.adc
        except OSError: return None

    def get_level(self):
        return int((self.adc.read() / self._MAX_ADC) * self._MAX_LVL)

class TemperatureSensor:
    """Gray water tank temperature sensor NTC10k"""
    def __init__(self):
        self._MAX_ADC = 4095
        self._MAX_V = 3.3
        self._NTC_A = 1.14782450e-03
        self._NTC_B = 2.31732771e-04
        self._NTC_C = 9.16549992e-08
        self._R_DIVIDER = 10000.0
        self._KELVINS_ZERO = -273.15
        self._TEMP_SENSOR_PIN = Pin(34, Pin.IN)
        self.adc = None

    def configure(self):
        try:
            self.adc = ADC(self._TEMP_SENSOR_PIN)
            self.adc.atten(ADC.ATTN_11DB)
            return self.adc
        except OSError: return None

    def get_temperature(self):
        Vntc = max((self.adc.read() * self._MAX_V) / self._MAX_ADC, 1e-10)
        Rntc = max(self._R_DIVIDER * ((self._MAX_V / Vntc) - 1), 1e-10)
        temp_k = (1 / (self._NTC_A + (self._NTC_B * log(Rntc)) + (self._NTC_C * pow(log(Rntc), 3.0))))
        return temp_k + self._KELVINS_ZERO


class Valve:
    """Electric valve regulation"""
    MOVE_TIME_SEC = 60  # valve movement time
    AUTO_CLOSE_SEC = 600  # auto-close time

    def __init__(self):
        self._VALVE_BTN = Pin(2, Pin.IN)
        self._OPEN_PIN = Pin(5, Pin.OUT)
        self._CLOSE_PIN = Pin(16, Pin.OUT)

        self._STATE = 0
        self._move_task = None
        self._auto_close_task = None

    def configure(self):
        self._OPEN_PIN.value(0)
        self._CLOSE_PIN.value(0)

    def get_state(self):
        return self._STATE

    def is_button_pressed(self):
        return self._VALVE_BTN.value() == 1

    async def _run_valve(self, state):
        try:
            # Ensure both pins are off first
            self._OPEN_PIN.value(0)
            self._CLOSE_PIN.value(0)

            if state == 1:
                self._OPEN_PIN.value(1)
                print("Valve OPENING")
            else:
                self._CLOSE_PIN.value(1)
                print("Valve CLOSING")

            await asyncio.sleep(self.MOVE_TIME_SEC)

        except asyncio.CancelledError:
            print("Valve movement cancelled")
            raise
        finally:
            # Always stop power to motor
            self._OPEN_PIN.value(0)
            self._CLOSE_PIN.value(0)
            print("Valve motor stopped")

    async def _auto_close(self):
        try:
            await asyncio.sleep(self.AUTO_CLOSE_SEC)

            if self._STATE == 1:
                print("Auto-closing valve after timeout")
                await self._start_valve_task(0)

        except asyncio.CancelledError:
            print("Auto-close cancelled")
            raise

    async def _start_valve_task(self, state):
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

        self._STATE = state

        # Start valve movement
        self._move_task = asyncio.create_task(self._run_valve(state))

        # If opening, schedule auto-close
        if state == 1:
            self._auto_close_task = asyncio.create_task(self._auto_close())

    async def control(self, ble):
        while True:
            try:
                # Physical button forces OPEN
                if self.is_button_pressed():
                    print("Physical button pressed. Forcing valve OPEN.")

                    await self._start_valve_task(1)
                    await ble.write_characteristic("VALVE", self.get_state(), '<B')

                    await ble.write_characteristic("VALVE_LOCKED", 1, '<B')

                    while self.is_button_pressed():
                        await asyncio.sleep_ms(50)

                    print("Physical button released.")
                    await ble.write_characteristic("VALVE_LOCKED", 0, '<B')
                    continue

                try:
                    _, data = await asyncio.wait_for_ms(ble.characteristics["VALVE"].written(), 50)
                    command = ble.decode_data(data)

                    if command is not None and command != self.get_state():
                        print(f"App command received: {command}")
                        await self._start_valve_task(command)

                except asyncio.TimeoutError:
                    pass

            except asyncio.CancelledError:
                print("Valve control task cancelled")

                # Stop any running movement
                if self._move_task is not None:
                    self._move_task.cancel()

                # Cancel auto close task
                if self._auto_close_task is not None:
                    self._auto_close_task.cancel()

                break

            except Exception as e:
                print("Error in valve control task:", e)
                await asyncio.sleep_ms(200)

class Heater:
    def __init__(self):
        self._HEATER_PIN = Pin(4, Pin.OUT)
        self._STATE = 0
        self._HEATING_TEMP = 4.5
        self._HYSTERESIS = 0.5

    def configure(self):
        self._HEATER_PIN.value(0)

    def control_heater(self, state):
        self._STATE = state
        self._HEATER_PIN.value(state)

    def get_state(self):
        return self._STATE

    async def control(self, sensor, ble):
        while True:
            try:
                temperature = sensor.get_temperature()
                current_state = self.get_state()

                new_state = current_state
                if temperature <= (self._HEATING_TEMP - self._HYSTERESIS):
                    new_state = 1 # Turn ON
                elif temperature >= (self._HEATING_TEMP + self._HYSTERESIS):
                    new_state = 0 # Turn OFF

                if new_state != current_state:
                    self.control_heater(new_state)
                    print(f"Heater state changed to: {new_state}")
                    await ble.write_characteristic("HEATER", new_state, '<B')

            except asyncio.CancelledError:
                print("Heater task cancelled")
                break
            except Exception as e:
                print("Error in heater control task:", e)
            finally:
                await asyncio.sleep(5)

class Bluetooth:
    def __init__(self):
        self._ADV_INTERVAL_MS = 250000
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
        }
        self.characteristics = {}

    def configure(self):
        aioble.security(
            bonding=True,
            mitm=True,
            le_secure=True,
            io_capability=aioble.IO_CAPABILITY_DISPLAY_ONLY,
        )

        def display_passkey(passkey):
            print("BLE Passkey:", passkey)

        aioble.set_passkey_callback(display_passkey)
        aioble.set_static_passkey(PASSKEY)

        ble_service = aioble.Service(self._MANAGER_SERVICE_UUID)
        for name in self.characteristic_uuids:
            if name == "VALVE":
                 self.characteristics[name] = aioble.Characteristic(
                    ble_service, self.characteristic_uuids[name], read=True, write=True, capture=True, notify=True
                )
            else:
                self.characteristics[name] = aioble.Characteristic(
                    ble_service, self.characteristic_uuids[name], read=True, notify=True
                )
        aioble.register_services(ble_service)
        print("BLE Service Registered")

    @staticmethod
    def decode_data(data):
        try:
            if data is not None and len(data) > 0:
                return int.from_bytes(data, 'little')
        except Exception as e:
            print(f"Error decoding data: {data}", e)
        return None

    async def write_characteristic(self, name, data, fmt):
        encoded_data = struct.pack(fmt, data)
        self.characteristics[name].write(encoded_data, send_update=True)

    async def write_characteristic_multiple(self, name, data_tuple, fmt):
        encoded_data = struct.pack(fmt, *data_tuple)
        self.characteristics[name].write(encoded_data, send_update=True)

    async def advertise(self):
        print("Starting BLE advertising...")
        while True:
            try:
                async with await aioble.advertise(
                        self._ADV_INTERVAL_MS,
                        name=self._BT_NAME,
                        services=[self._MANAGER_SERVICE_UUID]
                ) as connection:
                    print("Connection from", connection.device)
                    await self.write_characteristic("VALVE", valve.get_state(), '<B')
                    await self.write_characteristic("VALVE_LOCKED", 1 if valve.is_button_pressed() else 0, '<B')
                    await self.write_characteristic("HEATER", heater.get_state(), '<B')
                    await connection.disconnected()
                    print("Disconnected from", connection.device)
            except asyncio.CancelledError:
                print("Advertise task cancelled")
                break
            except Exception as e:
                print("Error in advertise task:", e)
            finally:
                await asyncio.sleep_ms(100)


ble = Bluetooth()
mpu = MPU()
oled = OLED()
gray_lvl_sensor = GrayLevelSensor()
clean_lvl_sensor = CleanLevelSensor()
temp_sensor = TemperatureSensor()
heater = Heater()
valve = Valve()

async def mpu_sensor_task():
    if not mpu.configure(): return
    while True:
        try:
            pitch, roll, yaw = mpu.get_rotation()
            await ble.write_characteristic_multiple(
                "MPU_SENSOR", (pitch, roll, yaw), '<3f')
        except Exception as e:
            print(f"Error in MPU task: {e}")
        await asyncio.sleep(1)

async def environmental_sensors_task():
    if not (gray_lvl_sensor.configure() and clean_lvl_sensor.configure() and temp_sensor.configure()):
        print("One or more environmental sensors failed to configure.")
        return
    while True:
        try:
            gray_lvl = gray_lvl_sensor.get_level()
            clean_lvl = clean_lvl_sensor.get_level()
            temp_celsius = temp_sensor.get_temperature()

            await ble.write_characteristic("GRAY_LEVEL_SENSOR", gray_lvl, '<B')
            await ble.write_characteristic("CLEAN_LEVEL_SENSOR", clean_lvl, '<B')
            await ble.write_characteristic("GRAY_TEMP_SENSOR", int(temp_celsius * 100), '<h')

            oled.print_tank_info(gray_lvl, clean_lvl, valve.get_state())

        except Exception as e:
            print(f"Error in environmental sensors task: {e}")
        await asyncio.sleep(1)


async def main():
    ble.configure()
    oled.configure()
    valve.configure()
    heater.configure()

    tasks = [
        asyncio.create_task(ble.advertise()),
        asyncio.create_task(environmental_sensors_task()),
        asyncio.create_task(mpu_sensor_task()),
        asyncio.create_task(heater.control(temp_sensor, ble)),
        asyncio.create_task(valve.control(ble)),
    ]
    await asyncio.gather(*tasks)

try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("Program stopped by user")
except Exception as e:
    print(f"An unexpected error occurred in main run: {e}")
