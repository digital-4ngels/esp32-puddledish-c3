# ESP32 S3 Puddle Dish

> Demo video coming soon. Drop it at [`assets/hero.gif`](assets/hero.gif).

A 3D particle fluid living inside a [Waveshare ESP32-S3-Touch-AMOLED-1.75-B](https://www.waveshare.com/esp32-s3-touch-amoled-1.75.htm) — round, 466×466. The screen is the front glass of a shallow dish. Tilt the board and the liquid pours to the rim; shake it and it sprays white.

**Puddle Dish is built on [FluidBox](https://github.com/V4C38/esp32-fluidbox)** by [V4C38](https://github.com/V4C38) ([Johannes Tscharn](https://x.com/JohannesTscharn)). FluidBox is the original: Clavet double-density solver, band renderer, IMU, PWR reset — written for the rectangular 1.8" Waveshare board. This repo ports that work onto the round 1.75-B. Do not flash the FluidBox 1.8 binary here.

Sister repo (Rust, experimental): [esp32-puddledish-rust](https://github.com/digital-4ngels/esp32-puddledish-rust). The C firmware in this repo is the one that feels right.

## What this is

Custom firmware for the 1.75-B. Nine hundred glowing particles slosh around inside a virtual dish shaped like the glass — move, tilt, or shake the board and they follow, as if liquid were trapped behind the screen.

This repo contains:

- A **fluid simulation** that runs on the ESP32-S3 in real time
- A **renderer** that draws each particle with perspective, depth, and velocity-based colour
- **IMU integration** so gravity, shake, and rotation all affect the fluid

Press the case **PWR** button briefly to reset the simulation. Holding PWR still powers the device off as usual. BOOT is left alone.

## How it works

- **Rendering** — the display is drawn in horizontal strips and sent out while the next strip is being prepared. Particle colour comes from a precomputed table based on speed and depth.
- **Simulation** — particles push each other apart when too close and pull together when too far (Clavet 2005), which keeps the motion stable even when you shake the board hard.
- **Motion** — the onboard accelerometer and gyroscope tell the simulation which way is down and how the board is moving.

The box is a flat cylinder: the circle is the glass, the depth is a 75 px can. Native panel +Y is physical right when USB is down, so projection is rotated 90° clockwise and gravity still means “down”.

For pins, first-time setup, and tunables, see [`firmware/README.md`](firmware/README.md).

## Running it

ESP-IDF **v5.5.5**. USB serial-JTAG on this board (Windows often `COM3`).

```bash
. $IDF_PATH/export.sh          # PowerShell: . C:\Users\<you>\esp\esp-idf\export.ps1
cd firmware
idf.py -p COM3 flash monitor   # macOS/Linux: -p /dev/cu.usbmodem* or /dev/ttyACM0
```

## Layout

| Path | Contents |
|---|---|
| `firmware/` | The ESP-IDF project: display, IMU, solver, renderer |
| `assets/` | Hero clip (add `hero.gif` when you have it) |

## Credits

- **[FluidBox](https://github.com/V4C38/esp32-fluidbox)** — V4C38 / Johannes Tscharn. Solver, renderer, IMU, and the idea that the screen is the front wall of a glass box. MIT.
- Waveshare — ESP32-S3-Touch-AMOLED-1.75-B, `esp_lcd_co5300`, `qmi8658`.

## License

[MIT](LICENSE). FluidBox code remains © 2026 V4C38. Our 1.75-B port is © 2026 digital-4ngels.
