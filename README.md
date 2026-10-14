# Tank-Manager

Controller for a gray and a clean water tank, powered from 12 V. An ESP32 measures the tank
levels, the gray water temperature and the tilt of the device, drives the gray water drain valve,
a tank heater and an overfill buzzer, and is operated from a front panel (OLED display and rotary
encoder) or from a phone app over Bluetooth LE.

This repository holds the hardware design and the firmware.

## Folders

| Folder | Contents |
| --- | --- |
| [`esp32/`](esp32/) | MicroPython firmware of the controller |
| [`PCB-design/`](PCB-design/) | KiCad project of the main board |
| [`PCB-front/`](PCB-front/) | KiCad project of the front panel board |
| [`temp_sensor/`](temp_sensor/) | Characterisation of the NTC temperature sensor |

### `esp32/`

The firmware: sensor readout, valve, heater and buzzer control, the display menu, Bluetooth LE
communication with the app and the MPU calibration. The components, pins, Bluetooth
characteristics and the calibration procedure are described in [`esp32/README.md`](esp32/README.md).

### `PCB-design/`

KiCad project of the main board (`PCB-design.kicad_sch`, `PCB-design.kicad_pcb`). It carries:

- the ESP32 DevKitC module,
- the 12 V input with an on/off switch, fuses and protection diodes, and a Pololu D24V10F3
  step-down regulator,
- power MOSFETs and PC817 optocouplers for the valve and heater outputs,
- the buzzer,
- the resistor dividers and connectors for the level and temperature sensors,
- a 9-pin connector to the front panel.

`Custom_components.pretty/` holds the footprints drawn for this project (ESP32 DevKitC,
MPU-9250, SH1106 OLED, Pololu D24V10F3); the front panel project uses them too.

### `PCB-front/`

KiCad project of the front panel board (`PCB-front.kicad_sch`, `PCB-front.kicad_pcb`): the
1.3" SH1106 OLED display, the EC12 rotary encoder, the MPU-9250 module and the 9-pin connector
to the main board.

### `temp_sensor/`

Data and a script used to find the temperature curve of the NTC 10k sensor:

- `ntc10k_nominal.csv`: nominal, minimum and maximum resistance of the thermistor from -40 °C,
- `r-t_curve.csv`: resistance measured on the actual sensor,
- `r-t_curve.py`: fits the Steinhart-Hart coefficients to the nominal data and plots them
  against the measurements. It needs `numpy`, `pandas`, `scipy` and `matplotlib`.

The fitted coefficients are the ones used by `TemperatureSensor` in `esp32/sensors.py`.

## License

MIT, see [`LICENSE`](LICENSE).
