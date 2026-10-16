import struct
import time


class History:
    """Level history for the app's charts: one sample (averaged readings) every 10 minutes, kept for
    30 days in a ring buffer file of fixed size. Samples taken while driving or standing tilted are
    flagged, because the levels read then are off (water sloshes or lies at a slant).

    Samples are numbered (sequence numbers grow forever, the slot is seq % CAPACITY), so the app asks
    only for what it does not have yet. The controller does not know the date: the app sends it with
    every request. Until then samples carry the seconds since start-up with the UNKNOWN_TIME flag, and
    those of the current start-up are corrected as soon as the date is known.
    """
    FILE = "history.bin"
    INTERVAL_S = 600
    CAPACITY = 30 * 24 * 6
    RECORD = "<IBBh"            # time (Unix seconds), gray %, clean %, gray temperature in degC x 100
    # Top bits of the level bytes (levels are 0-100): the readings of that sample are not reliable
    MOVING = 0x80               # on the gray level: the camper was driving
    TILTED = 0x80               # on the clean level: the camper stood tilted
    RECORD_SIZE = 8
    UNKNOWN_TIME = 0x80000000   # flag in the time field: seconds since start-up, the date was not known
    INVALID_TEMP = -32768
    _MAGIC = b"TMH1"
    _HEADER = "<4sII"           # magic, capacity, next sequence number
    _HEADER_SIZE = 12
    _TIME_TOLERANCE_S = 120     # a new date from the app is taken only if the clock is off by more
    # Once dated after start-up, only corrections up to this are taken (the ESP32 clock drifts a few
    # seconds a day); a bigger jump comes from a phone or tablet with a wrong clock and is ignored
    _MAX_CORRECTION_S = 600
    # Driving: the acceleration changes by more than MOTION_DELTA (m/s^2, ~0.1 g) between two
    # readings (1 s apart) at least MOTION_COUNT times in a sample. Tilted: average pitch or roll
    # beyond TILT_DEG. Both need a calibrated MPU; without one no sample is flagged.
    MOTION_DELTA = 1.0
    MOTION_COUNT = 5
    TILT_DEG = 3.0

    def __init__(self):
        self.next_seq = 0
        self._boot = time.time()
        self._boot_first_seq = 0
        self._offset = None         # Unix time minus time.time(), once the app has told the date
        self._start_window()

    def configure(self):
        try:
            with open(self.FILE, "rb") as f:
                magic, capacity, next_seq = struct.unpack(self._HEADER, f.read(self._HEADER_SIZE))
            if magic != self._MAGIC or capacity != self.CAPACITY:
                raise ValueError("other format")
            self.next_seq = next_seq
            print("History:", min(next_seq, self.CAPACITY), "samples")
        except (OSError, ValueError) as e:
            print("Creating the history file:", e)
            self.next_seq = 0
            zeros = bytes(512)
            with open(self.FILE, "wb") as f:
                f.write(struct.pack(self._HEADER, self._MAGIC, self.CAPACITY, 0))
                left = self.CAPACITY * self.RECORD_SIZE
                while left > 0:
                    f.write(zeros[:min(left, len(zeros))])
                    left -= len(zeros)
        self._boot_first_seq = self.next_seq

    def oldest_seq(self):
        return max(0, self.next_seq - self.CAPACITY)

    # ------------------------------------------------------------------ sampling

    def _start_window(self):
        self._window_start = time.time()
        self._n = 0
        self._gray = 0
        self._clean = 0
        self._temp_n = 0
        self._temp = 0
        self._motion = 0
        self._tilt_n = 0
        self._pitch = 0.0
        self._roll = 0.0
        self._last_acc = None

    def add_motion(self, acc, pitch, roll):
        """Called with every MPU reading (about once a second): acceleration vector in m/s^2 and the
        angles relative to the level set from the menu"""
        last = self._last_acc
        if last is not None:
            dx, dy, dz = acc[0] - last[0], acc[1] - last[1], acc[2] - last[2]
            if dx * dx + dy * dy + dz * dz > self.MOTION_DELTA * self.MOTION_DELTA:
                self._motion += 1
        self._last_acc = acc
        self._tilt_n += 1
        self._pitch += pitch
        self._roll += roll

    def _flags(self):
        moving = self._motion >= self.MOTION_COUNT
        tilted = self._tilt_n > 0 and max(abs(self._pitch), abs(self._roll)) / self._tilt_n > self.TILT_DEG
        return (self.MOVING if moving else 0), (self.TILTED if tilted else 0)

    def add_reading(self, gray, clean, temp_celsius):
        """Called with every sensor reading (about once a second); stores the average every INTERVAL_S"""
        self._n += 1
        self._gray += gray
        self._clean += clean
        if temp_celsius is not None:
            self._temp_n += 1
            self._temp += temp_celsius
        if time.time() - self._window_start < self.INTERVAL_S:
            return
        temp = int(self._temp * 100 / self._temp_n) if self._temp_n else self.INVALID_TEMP
        moving, tilted = self._flags()
        record = struct.pack(self.RECORD, self._now(), min(100, round(self._gray / self._n)) | moving,
                             min(100, round(self._clean / self._n)) | tilted, temp)
        last_acc = self._last_acc
        self._start_window()
        self._last_acc = last_acc   # the next window compares its first reading with this one
        try:
            with open(self.FILE, "r+b") as f:
                self._write_record(f, self.next_seq, record)
                self.next_seq += 1
                f.seek(0)
                f.write(struct.pack(self._HEADER, self._MAGIC, self.CAPACITY, self.next_seq))
        except OSError as e:
            print("Saving the history failed:", e)

    def _now(self):
        if self._offset is None:
            return self.UNKNOWN_TIME | (time.time() - self._boot)
        return time.time() + self._offset

    def _write_record(self, f, seq, record):
        f.seek(self._HEADER_SIZE + (seq % self.CAPACITY) * self.RECORD_SIZE)
        f.write(record)

    # ------------------------------------------------------------------ for the app

    def read(self, seq, count):
        """Records seq .. seq + count - 1, as stored"""
        out = bytearray()
        with open(self.FILE, "rb") as f:
            for s in range(seq, seq + count):
                f.seek(self._HEADER_SIZE + (s % self.CAPACITY) * self.RECORD_SIZE)
                out += f.read(self.RECORD_SIZE)
        return bytes(out)

    def set_time(self, unix):
        """Date from the app. The first one after start-up also dates the samples taken without it."""
        offset = unix - time.time()
        if self._offset is not None:
            off_by = abs(offset - self._offset)
            if off_by <= self._TIME_TOLERANCE_S:
                return
            if off_by > self._MAX_CORRECTION_S:
                print("Ignoring a date", offset - self._offset, "s off from the clock")
                return
        first = self._offset is None
        self._offset = offset
        if not first:
            return
        fixed = 0
        with open(self.FILE, "r+b") as f:
            for seq in range(max(self._boot_first_seq, self.oldest_seq()), self.next_seq):
                f.seek(self._HEADER_SIZE + (seq % self.CAPACITY) * self.RECORD_SIZE)
                t, gray, clean, temp = struct.unpack(self.RECORD, f.read(self.RECORD_SIZE))
                if t & self.UNKNOWN_TIME:
                    t = self._boot + (t & ~self.UNKNOWN_TIME) + offset
                    self._write_record(f, seq, struct.pack(self.RECORD, t, gray, clean, temp))
                    fixed += 1
        print("Date set from the app;", fixed, "samples dated")
