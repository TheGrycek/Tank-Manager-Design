from machine import Pin, I2C
import utime

# Inicjalizacja I2C
i2c = I2C(scl=Pin(22), sda=Pin(21), freq=100000)

# Adres urządzenia (0x68)
MPU_ADDR = 0x68

# Funkcja do odczytu rejestru
def read_register(register, n_bytes=1):
    return i2c.readfrom_mem(MPU_ADDR, register, n_bytes)

# Funkcja do zapisu w rejestrze
def write_register(register, value):
    i2c.writeto_mem(MPU_ADDR, register, bytes([value]))

# Sprawdzenie, czy czujnik jest dostępny
whoami = read_register(0x75)
print(f"WHOAMI: {whoami[0]:#04x}")  # Odczyt WHO_AM_I

if whoami[0] == 0x73:
    print("MPU6500 jest podłączony.")
else:
    print("Błąd: Nie znaleziono MPU6500!")

# Resetowanie MPU (PWR_MGMT_1)
write_register(0x6B, 0x80)  # Reset
utime.sleep(0.1)
write_register(0x6B, 0x00)  # Wyłącz tryb uśpienia

# Odczyt akcelerometru
accel_x = read_register(0x3B, 2)  # Odczyt z rejestru dla X
accel_y = read_register(0x3D, 2)  # Odczyt z rejestru dla Y
accel_z = read_register(0x3F, 2)  # Odczyt z rejestru dla Z

# Przekształcenie wyników na liczby całkowite
accel_x = int.from_bytes(accel_x, 'big')
accel_y = int.from_bytes(accel_y, 'big')
accel_z = int.from_bytes(accel_z, 'big')

print(f"Akcelerometr: X={accel_x}, Y={accel_y}, Z={accel_z}")