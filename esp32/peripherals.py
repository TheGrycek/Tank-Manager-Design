import asyncio
import time

from machine import ADC, Pin, SoftI2C, disable_irq, enable_irq

from sh1106 import SH1106_I2C


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
        # False while the display is switched off to save power (ui_task)
        self.on = True
        self.valve_states = {
            0: "closed",
            1: "open",
            2: "closing",
            3: "opening"
        }
        self.valve_errors = {
            Valve.ERROR_NO_CURRENT: "NO CURRENT",
            Valve.ERROR_TIMEOUT: "TIMEOUT"
        }

    def configure(self):
        try:
            self.i2c = SoftI2C(scl=self._OLED_I2C_SCL_PIN, sda=self._OLED_I2C_SDA_PIN)
            # Rotated by 180 degrees: the display is mounted upside down on the front panel
            self.oled = SH1106_I2C(self._OLED_WIDTH, self._OLED_HEIGHT, self.i2c, addr=self._OLED_ADDR,
                                   rotate=180)
            self.oled.contrast(255)
            self.oled.fill(0)
            self.oled.text("System Ready", 0, 0)
            self.oled.show()
            return self.oled
        except OSError:
            print("Error: OLED not found")
            self.oled = None
            return None

    def set_on(self, on):
        """Switches the display panel on or off; the picture is kept"""
        self.on = on
        if not self.oled: return
        try:
            if on:
                self.oled.poweron()
            else:
                self.oled.poweroff()
        except OSError:
            print("Error: OLED not responding")

    def print_tank_info(self, level_gray, level_clean, valve_state, temperature, valve_error, blink):
        if not self.oled: return
        temp = "T ERR" if temperature is None else f"{temperature:.1f}C"
        self.oled.fill(0)
        self.oled.text(f"|GRAY TANK>", 0, 0)
        self.oled.text(f" lvl:{level_gray:3d}% {temp}", 0, 15)
        if valve_state != Valve.ERROR:
            self.oled.text(f" valve: {self.valve_states[valve_state]}", 0, 25)
        elif blink:
            # Inverted bar, alternating with the cause
            self.oled.fill_rect(0, 24, self._OLED_WIDTH, 10, 1)
            self.oled.text(" VALVE ERROR", 0, 25, 0)
        else:
            self.oled.text(f" {self.valve_errors.get(valve_error, '')}", 0, 25)
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

    def print_calibration(self, step, line1, line2, line3):
        if not self.oled: return
        self.oled.fill(0)
        self.oled.text(f"|MPU CAL {step}>" if step else "|MPU CAL>", 0, 0)
        self.oled.text(line1, 0, 20)
        self.oled.text(line2, 0, 32)
        self.oled.text(line3, 0, 50)
        self.oled.show()

    def print_passkey(self, passkey):
        if not self.oled: return
        self.oled.fill(0)
        self.oled.text("|BLUETOOTH>", 0, 0)
        self.oled.text(" pairing code:", 0, 20)
        self.oled.text(f"   {passkey:06d}", 0, 40)
        self.oled.show()

class Valve:
    """Electric valve regulation. The motor runs until its limit switch cuts the current; the time
    limit is only a safeguard. A move without any current, or one that does not end in time, stops
    the motor and leaves the valve in the ERROR state until the next command."""
    ERROR = 4  # state after a failed move
    ERROR_NO_CURRENT = 1  # the motor drew no current: wire, fuse F3 or MOSFET
    ERROR_TIMEOUT = 2  # the motor still ran after MAX_MOVE_MS: valve stuck or limit switch broken
    # Measured without water pressure: about 22 s per move at 15-45 mA (up to 100 mA at the start);
    # under pressure the motor is slower
    MAX_MOVE_MS = 40000
    _START_MS = 1000  # the current must appear within this
    _STOP_MS = 500  # the move has ended after this long without current
    _POLL_MS = 20
    # The ADC reads 142 mV at 0 V (the bottom of its range); the motor gives 150-450 mV on the shunt
    _CURRENT_MIN_UV = 160000
    AUTO_CLOSE_SEC = 600  # auto-close time

    def __init__(self):
        self._OPEN_PIN = Pin(13, Pin.OUT)
        self._CLOSE_PIN = Pin(27, Pin.OUT)
        # Motor current: 10 ohm shunt (R17) in the valve return, RC filtered to GPIO33, so 10 mV = 1 mA
        # (the motor was measured at about 30 mA while moving)
        self._current_adc = ADC(Pin(33))
        self._current_adc.atten(ADC.ATTN_11DB)

        self._STATE = 0 # 0-closed, 1-open, 2-closing, 3-opening, 4-error
        self._target = 0  # last commanded position, 1 = open
        self.error = None  # ERROR_* while the state is ERROR
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
        # What the app shows: 1 when open or opening (or failed while opening), 0 otherwise
        return self._target

    def current_ma(self):
        """Valve motor current in mA"""
        uv = self._current_adc.read_uv()
        return 0 if uv < self._CURRENT_MIN_UV else uv // 10000

    async def _move_wait(self):
        """Waits until the limit switch cuts the motor; returns None or an ERROR_* value"""
        start = time.ticks_ms()
        last_current = None
        peak = 0
        while True:
            await asyncio.sleep_ms(self._POLL_MS)
            now = time.ticks_ms()
            elapsed = time.ticks_diff(now, start)
            current = self.current_ma()
            if current:
                last_current = now
                peak = max(peak, current)
            if last_current is None:
                if elapsed >= self._START_MS:
                    return self.ERROR_NO_CURRENT
            elif time.ticks_diff(now, last_current) >= self._STOP_MS:
                # Short drops to zero while the motor runs last one sample at most
                print(f"Valve moved in {time.ticks_diff(last_current, start) / 1000:.1f} s, peak {peak} mA")
                return None
            if elapsed >= self.MAX_MOVE_MS:
                return self.ERROR_TIMEOUT

    async def _set_state(self, state, ble):
        self._STATE = state
        # The app shows the movement (closing / opening) as well
        await ble.write_characteristic("VALVE_STATE", state, '<B')

    async def _run_valve(self, state, ble, start_up):
        try:
            # Ensure both pins are off first
            self.stop()

            self.error = None
            if state == 1:
                self._OPEN_PIN.value(1)
                print("Valve OPENING")
                await self._set_state(3, ble)
            else:
                self._CLOSE_PIN.value(1)
                print("Valve CLOSING")
                await self._set_state(2, ble)
            error = await self._move_wait()
            self.stop()
            if error == self.ERROR_NO_CURRENT and start_up:
                # After a start-up the valve is usually closed already, so its limit switch is open
                print("Valve already closed (no motor current)")
                error = None
            if error is None:
                await self._set_state(state, ble)
            else:
                print("Valve ERROR:", "no motor current" if error == self.ERROR_NO_CURRENT
                      else f"motor still running after {self.MAX_MOVE_MS // 1000} s")
                self.error = error
                await self._set_state(self.ERROR, ble)

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
        # Also after a failed opening: the valve may be partly open
        if self._target == 1:
            print("Auto-closing valve after timeout")
            await self._start_valve_task(0, ble)
            await ble.write_characteristic("VALVE", 0, '<B')

    async def _start_valve_task(self, state, ble, start_up=False):
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
        self._target = state
        self._move_task = asyncio.create_task(self._run_valve(state, ble, start_up))

        # If opening, schedule auto-close
        if state == 1:
            self._auto_close_task = asyncio.create_task(self._auto_close(ble))

    async def close_on_start(self, ble):
        """After a reset the real position is unknown (the valve may have been left open): drive it
        closed; if it already is, its limit switch is open and no current flows"""
        print("Closing the valve after start-up")
        await self._start_valve_task(0, ble, start_up=True)

    async def request(self, state, ble):
        """Open (1) or close (0) the valve; used by the app command and by the panel menu"""
        s = self.get_state()
        # After an error either command is a new attempt
        if (state == 0 and s in (1, 3)) or (state == 1 and s in (0, 2)) or s == self.ERROR:
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
        self._HEATER_PIN = Pin(25, Pin.OUT)
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
    """Valve error alarm (three beeps every 2 s), gray tank overfill alarm (a double beep every second)
    and the clean tank fill mode (beeps that come faster as the tank fills: silent when empty, a
    continuous tone when full)"""
    _BEEP_MS = 100
    _GAP_MS = 100
    _PERIOD_MS = 1000
    _VALVE_PAUSE_MS = 1500
    # Fill mode: silent up to FILL_EMPTY %, continuous from FILL_FULL %, in between a short beep with
    # a pause shrinking linearly from FILL_PAUSE_MAX_MS to FILL_PAUSE_MIN_MS
    FILL_EMPTY = 5
    FILL_FULL = 95
    _FILL_BEEP_MS = 60
    _FILL_PAUSE_MAX_MS = 1200
    _FILL_PAUSE_MIN_MS = 60

    def __init__(self):
        self._BUZZER_PIN = Pin(26, Pin.OUT)
        self._valve_alarm = False
        self._alarm = False
        self._fill_mode = False
        self._clean_lvl = 0

    def configure(self):
        self.off()

    def off(self):
        self._valve_alarm = False
        self._alarm = False
        self._fill_mode = False
        self._BUZZER_PIN.value(0)

    def control(self, valve_state, gray_lvl, clean_lvl, settings):
        # Sound while the gray tank is above the threshold and the valve is still closed
        self._alarm = settings.buzzer_on and valve_state == 0 and gray_lvl > settings.gray_lvl_threshold
        # A failed valve move: three beeps every 2 s, before everything else
        self._valve_alarm = settings.buzzer_on and valve_state == Valve.ERROR
        self._fill_mode = settings.fill_mode
        self._clean_lvl = clean_lvl

    @classmethod
    def fill_pause_ms(cls, level):
        """Pause between the fill mode beeps; None = silent, 0 = continuous"""
        if level <= cls.FILL_EMPTY:
            return None
        if level >= cls.FILL_FULL:
            return 0
        fraction = (level - cls.FILL_EMPTY) / (cls.FILL_FULL - cls.FILL_EMPTY)
        return int(cls._FILL_PAUSE_MAX_MS - fraction * (cls._FILL_PAUSE_MAX_MS - cls._FILL_PAUSE_MIN_MS))

    async def run(self):
        """Plays the fill mode beeps or, if it is off, the alarm"""
        pause_ms = self._PERIOD_MS - 2 * self._BEEP_MS - self._GAP_MS
        try:
            while True:
                if self._valve_alarm:
                    for gap_ms in (self._GAP_MS, self._GAP_MS, self._VALVE_PAUSE_MS):
                        self._BUZZER_PIN.value(1)
                        await asyncio.sleep_ms(self._BEEP_MS)
                        self._BUZZER_PIN.value(0)
                        await asyncio.sleep_ms(gap_ms)
                    continue
                if self._fill_mode:
                    fill_pause = self.fill_pause_ms(self._clean_lvl)
                    if fill_pause is None or fill_pause == 0:
                        self._BUZZER_PIN.value(0 if fill_pause is None else 1)
                        await asyncio.sleep_ms(50)
                        continue
                    self._BUZZER_PIN.value(1)
                    await asyncio.sleep_ms(self._FILL_BEEP_MS)
                    self._BUZZER_PIN.value(0)
                    await asyncio.sleep_ms(fill_pause)
                    continue
                if not self._alarm:
                    self._BUZZER_PIN.value(0)
                    await asyncio.sleep_ms(50)
                    continue
                for gap_ms in (self._GAP_MS, pause_ms):
                    self._BUZZER_PIN.value(1)
                    await asyncio.sleep_ms(self._BEEP_MS)
                    self._BUZZER_PIN.value(0)
                    await asyncio.sleep_ms(gap_ms)
        finally:
            self._BUZZER_PIN.value(0)


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
