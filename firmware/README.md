# Puddle Dish firmware

ESP-IDF C firmware for the Waveshare **ESP32-S3-Touch-AMOLED-1.75-B** (round 466×466, CO5300).

This is a port of [FluidBox](https://github.com/V4C38/esp32-fluidbox) (V4C38 / Johannes Tscharn) from the rectangular 1.8" board. Solver and renderer come from there. Display bring-up and IMU map are for this panel.

Do not flash the FluidBox 1.8 binary onto a 1.75-B.

## Getting it running

```bash
. $IDF_PATH/export.sh          # PowerShell: . C:\Users\<you>\esp\esp-idf\export.ps1
cd firmware
idf.py -p COM3 flash monitor
```

Press `Ctrl-]` to leave the monitor.

### First-time setup

ESP-IDF v**5.5.5**:

```bash
mkdir -p ~/esp && cd ~/esp
git clone -b v5.5.5 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf && ./install.sh esp32s3
```

On Windows, use System Python 3.12 for `export.ps1` / `install.ps1` — not a project venv.

`export.sh` / `export.ps1` has to be sourced in each new shell.

### How flashing actually works

The ESP32-S3 USB peripheral is wired to the Type-C port. Console is **USB serial-JTAG** (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG`), not UART0. On Windows that is usually `COM3`.

**If flashing ever fails**, hold **BOOT**, plug the cable in, release. That starts ROM download mode. This project never touches GPIO0.

### Dependencies

`main/idf_component.yml` pulls `espressif/esp_lcd_co5300` and `waveshare/qmi8658` from the Espressif component registry on first build.

### This board, not the 1.8

| | FluidBox 1.8 | Puddle Dish 1.75-B |
|---|---|---|
| Glass | 368×448 rectangle | 466×466 circle |
| Display | SH8601 or CO5300 V2 | CO5300 |
| Pins | 1.8 Waveshare map | PCLK 38, CS 12, RST 39, D0–D3 = 4/5/6/7, gap x=6 |
| I2C | board default | SDA 15 / SCL 14 |
| IMU map | Johannes 1.8 | `-ax, -ay, az` on this board |
| QSPI | 80 MHz in FluidBox | 40 MHz (known good here) |
| Bands | 16×28 | 22 rows, last band 4 (21×22+4) |

## What you should see

900 particles pool at the rim. Tilt pours. Shake brightens the spray. Short **PWR** reseeds a puddle. Long PWR still powers off in hardware.

## Layout

| File | Role |
|---|---|
| `main/main.c` | Startup and the two tasks |
| `main/config.h` | Every tunable constant |
| `main/display.c` | Panel bring-up, band buffers, DMA |
| `main/render.c` | Projection, colour tables, particle rasteriser |
| `main/sim.c` | FluidBox solver, grid sized for 466 |
| `main/imu.c` | QMI8658, gravity / shake |
| `main/button.c` | PWR via the TCA9554 |

Simulation is pinned to core 1, renderer to core 0. They meet at a short particle snapshot.

Everything adjustable is in [`main/config.h`](main/config.h).
