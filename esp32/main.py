import asyncio
import time

from machine import WDT, reset

import aioble
import aioble.core

# Start Bluetooth (loading the stored bonds) before the larger modules are compiled: compiling
# them grows the Python heap into the memory the Bluetooth controller needs, and its init then fails
aioble.core.ensure_active()

from peripherals import OLED, Valve, Heater, Buzzer, Encoder
from sensors import MPU, GrayLevelSensor, CleanLevelSensor, TemperatureSensor
from setup import Settings, Menu, Bluetooth


# Hardware watchdog timeout in ms, 0 = disabled.
# Once started it cannot be stopped (not even by Ctrl-C or a soft reset), so keep it at 0 while
# developing over the REPL and set it (e.g. 8000) for the installed device.
WATCHDOG_MS = 0

mpu = MPU()
oled = OLED()
gray_lvl_sensor = GrayLevelSensor()
clean_lvl_sensor = CleanLevelSensor()
temp_sensor = TemperatureSensor()
heater = Heater()
valve = Valve()
buzzer = Buzzer()
settings = Settings()
ble = Bluetooth(settings, valve, heater)
encoder = Encoder()
menu = Menu(settings, valve, ble, mpu)
# Latest sensor readings, shared between the sensor task and the display task
readings = {"gray": 0, "clean": 0, "temp": None}

async def mpu_sensor_task():
    while True:
        try:
            if mpu.calibrating:
                # Readings are raw during the calibration
                await asyncio.sleep(1)
                continue
            if mpu.mpu is None and not mpu.configure():
                # Sensor missing: keep trying, it may only be a loose ribbon
                await asyncio.sleep(10)
                continue
            if mpu.calibrated:
                # Without a calibration the angles are wrong, so nothing is sent to the app
                pitch, roll, yaw = mpu.get_rotation()
                await ble.write_characteristic_multiple(
                    "MPU_SENSOR", (pitch, roll, yaw), '<3f')
            else:
                # Still check that the sensor answers, so a lost one is noticed and set up again
                mpu.mpu.acceleration
                mpu.mpu.magnetic
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
            if settings.fill_mode_expired():
                settings.update(fill_mode=False)
                print("Fill mode off after", Settings.FILL_MODE_MAX_MS // 60000, "min")
                await ble.send_settings()
            buzzer.control(valve.get_state(), gray_lvl, clean_lvl, settings)

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
            elif settings.update(data[0], data[1], data[2], data[3], data[4] if len(data) > 4 else None):
                print("Settings changed from the app")
                # Start or stop the fill mode beeps at once, not with the next sensor reading
                buzzer.control(valve.get_state(), readings["gray"], readings["clean"], settings)
            # Always answer with the values actually in force (they may have been clamped)
            await ble.send_settings()
        except asyncio.CancelledError:
            break
        except Exception as e:
            print(f"Error in settings task: {e}")
            await asyncio.sleep_ms(200)

async def ui_task():
    # The only task that draws on the display: pairing code > calibration > menu > tank screen
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
                elif menu.state == Menu.CALIBRATE:
                    draw = ("calibration",) + menu.calibration_screen()
                elif menu.state != Menu.IDLE:
                    draw = ("menu",) + menu.screen()
                else:
                    draw = ("tank", readings["gray"], readings["clean"], valve.get_state(), readings["temp"])
                if draw != last_draw:
                    last_draw = draw
                    if draw[0] == "passkey":
                        oled.print_passkey(draw[1])
                    elif draw[0] == "calibration":
                        oled.print_calibration(*draw[1:])
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
    await valve.close_on_start(ble)

    tasks = [
        asyncio.create_task(ble.advertise()),
        asyncio.create_task(environmental_sensors_task()),
        asyncio.create_task(mpu_sensor_task()),
        asyncio.create_task(heater.control(temp_sensor, ble, settings)),
        asyncio.create_task(valve.control(ble)),
        asyncio.create_task(settings_task()),
        asyncio.create_task(ui_task()),
        asyncio.create_task(buzzer.run()),
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
