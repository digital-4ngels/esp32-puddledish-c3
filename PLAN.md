# Fluidbox 1.75-B Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Johannes’ 3D-Partikel-Flüssigkeit auf dem Waveshare ESP32-S3-Touch-AMOLED-1.75-B (466×466 rund), Schicht für Schicht.

**Architecture:** Neues ESP-IDF-C-Projekt in `esp32-puddledish-c3/firmware/`. Display und Streifen vom Orb (dieses Board). Solver und Renderer aus `repo/fluidbox/main/`. Zwei Tasks (Sim Core 1, Render Core 0), Treffpunkt eine Partikel-Kopie. Die Box ist ein flacher Zylinder — Kreis = Glas, Tiefe = Dose. `repo/` nicht flashen.

**Tech Stack:** ESP-IDF v5.5.5, C, `espressif/esp_lcd_co5300`, später `waveshare/qmi8658`, Board COM3, USB-Serial-JTAG.

## Global Constraints

- Board: Waveshare ESP32-S3-Touch-AMOLED-1.75-B, 466×466, COM3, MAC zuletzt `28:84:85:57:40:8c`
- Nicht das 1.8er-Binary aus `repo/` flashen
- IDF: `C:\Users\d4\esp\esp-idf`, vor `export.ps1` Python 3.12 an PATH-Anfang, `VIRTUAL_ENV` leeren
- Console: `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` (sonst Monitor leer)
- QSPI Layer 1: 40 MHz (Orb, bekannt gut). Nicht 80 MHz starten
- Pins Display: PCLK 38, CS 12, RST 39, D0–D3 = 4/5/6/7, Gap x=6
- I2C (ab Task 4): SDA 15, SCL 14
- BOOT (GPIO0) nicht als App-Taste, nicht umkonfigurieren
- PWR nur Bit 4 am TCA9554 lesen, restliche Expander-Pins nicht umbiegen
- 900 Partikel, `BOX_D = 75`, Kreis über `BOX_CORNER_R = BOX_W/2`
- Kein Touch, kein Audio, kein Rust, nichts aus `OBD2 ESP32`
- Dieses Workspace hat **kein Git** — Commit-Steps überspringen, nicht `git init`
- Nächste Schicht erst nach sichtbarem OK auf dem Glas
- PowerShell: Befehle mit `;` ketten, nicht `&&`

## File map

```text
esp32-puddledish-c3/firmware/
  CMakeLists.txt
  sdkconfig.defaults
  main/
    CMakeLists.txt
    idf_component.yml
    config.h
    main.c
    display.c / display.h     Task 1, bleibt
    pattern.c / pattern.h     Task 1, Layer-1-Muster; später ungenutzt
    lattice.c / lattice.h     Task 2 Stub; Task 3 entfernt
    render.c / render.h       Task 2, aus repo + Patches
    sim.c / sim.h             Task 3, aus repo + Grid/config
    imu.c / imu.h             Task 4, aus repo
    button.c / button.h       Task 4, aus repo
```

`repo/` nur lesen.

---

### Task 1: Panel lebt (Schicht 1)

**Files:**
- Create: `esp32-puddledish-c3/firmware/CMakeLists.txt`
- Create: `esp32-puddledish-c3/firmware/sdkconfig.defaults`
- Create: `esp32-puddledish-c3/firmware/main/CMakeLists.txt`
- Create: `esp32-puddledish-c3/firmware/main/idf_component.yml`
- Create: `esp32-puddledish-c3/firmware/main/config.h`
- Create: `esp32-puddledish-c3/firmware/main/display.h`
- Create: `esp32-puddledish-c3/firmware/main/display.c`
- Create: `esp32-puddledish-c3/firmware/main/pattern.h`
- Create: `esp32-puddledish-c3/firmware/main/pattern.c`
- Create: `esp32-puddledish-c3/firmware/main/main.c`
- Create: `esp32-puddledish-c3/firmware/README.md`

**Interfaces:**
- Consumes: nichts
- Produces:
  - `esp_err_t display_init(void)`
  - `uint16_t *display_acquire_band(void)` — Block, Puffer `LCD_H_RES * BAND_ROWS`
  - `esp_err_t display_flush_band(int y0, int rows, const uint16_t *buffer)`
  - `void display_frame(void (*draw)(uint16_t *band, int y0, int rows))`
  - `void pattern_draw_band(uint16_t *band, int y0, int rows)` — Kreuz + Ring
  - `config.h`: `LCD_H_RES 466`, `LCD_V_RES 466`, `LCD_COL_OFFSET 6`, `BAND_ROWS 22`, Display-Pins wie oben

- [x] **Step 1: IDF-Projekt anlegen**

`firmware/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(fluidbox)
```

`firmware/main/CMakeLists.txt`:

```cmake
idf_component_register(
    SRCS "main.c" "display.c" "pattern.c"
    INCLUDE_DIRS "."
)
```

`firmware/main/idf_component.yml`:

```yaml
dependencies:
  idf: ">=5.5"
  espressif/esp_lcd_co5300: "^2.1.0"
```

`firmware/sdkconfig.defaults`:

```
CONFIG_IDF_TARGET="esp32s3"
CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
CONFIG_ESPTOOLPY_FLASHMODE_QIO=y
CONFIG_ESPTOOLPY_FLASHFREQ_80M=y
CONFIG_PARTITION_TABLE_SINGLE_APP_LARGE=y
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_SPEED_80M=y
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=240
CONFIG_COMPILER_OPTIMIZATION_PERF=y
CONFIG_FREERTOS_HZ=1000
CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192
CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y
```

`firmware/README.md` (kurz): Projekt für 1.75-B, Flash COM3, `repo/` nicht flashen, vier Schichten siehe `../DESIGN.md`.

- [x] **Step 2: config.h + Display (Orb-Pins, Orb-API mit variablem letzten Streifen)**

`firmware/main/config.h`:

```c
#pragma once

#define LCD_H_RES 466
#define LCD_V_RES 466
#define LCD_COL_OFFSET 6

#define LCD_PCLK GPIO_NUM_38
#define LCD_CS GPIO_NUM_12
#define LCD_RST GPIO_NUM_39
#define LCD_D0 GPIO_NUM_4
#define LCD_D1 GPIO_NUM_5
#define LCD_D2 GPIO_NUM_6
#define LCD_D3 GPIO_NUM_7

#define BAND_ROWS 22
#define BAND_COUNT ((LCD_V_RES + BAND_ROWS - 1) / BAND_ROWS)

#define DISPLAY_BRIGHTNESS 230
```

`firmware/main/display.h`:

```c
#pragma once

#include "esp_err.h"
#include <stdint.h>

esp_err_t display_init(void);
uint16_t *display_acquire_band(void);
esp_err_t display_flush_band(int y0, int rows, const uint16_t *buffer);
void display_frame(void (*draw)(uint16_t *band, int y0, int rows));
```

`firmware/main/display.c`: Orb-`display.c` als Vorlage (`jxyhelper-agent-ui/firmware/main/display.c`).

Pflicht-Unterschiede zu Orb:
- `#include "orb.h"` streichen
- `display_frame` nimmt den `draw`-Callback statt `orb_draw_band`
- 40 MHz lassen (`pclk_hz = 40 * 1000 * 1000`)
- Gap `LCD_COL_OFFSET` (6)
- Log: `466x466 CO5300 ready`

Kern der Frame-Schleife:

```c
void display_frame(void (*draw)(uint16_t *band, int y0, int rows))
{
    for (int y = 0; y < LCD_V_RES; y += BAND_ROWS) {
        int rows = BAND_ROWS;
        if (y + rows > LCD_V_RES) {
            rows = LCD_V_RES - y;
        }
        uint16_t *band = display_acquire_band();
        draw(band, y, rows);
        ESP_ERROR_CHECK(display_flush_band(y, rows, band));
    }
}
```

Rest (SPI-Bus, CO5300, zwei DMA-Bänder, Counting-Semaphore) 1:1 vom Orb, nur ohne `orb.h`.

- [x] **Step 3: Testmuster Kreuz + Ring**

`firmware/main/pattern.h`:

```c
#pragma once
#include <stdint.h>
void pattern_draw_band(uint16_t *band, int y0, int rows);
```

`firmware/main/pattern.c`: schwarzer Grund. Weißes Kreuz durch die Mitte (x=233 und y=233, 3 px dick). Heller Ring Radius 220, Dicke 4 px. Farbe wie Orb byte-swapped RGB565:

```c
static uint16_t rgb565(int r, int g, int b)
{
    uint16_t c = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
    return (uint16_t)((c << 8) | (c >> 8));
}
```

Weiß = `rgb565(255,255,255)`. Pixel nur setzen wenn `(x-233)^2+(y-233)^2 <= 233^2` — außerhalb des Kreises schwarz lassen (Bezel).

`firmware/main/main.c`:

```c
#include "display.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pattern.h"

static const char *TAG = "fluidbox";

void app_main(void)
{
    ESP_LOGI(TAG, "FluidBox layer 1");
    ESP_ERROR_CHECK(display_init());
    for (;;) {
        display_frame(pattern_draw_band);
        vTaskDelay(1);
    }
}
```

- [x] **Step 4: Build**

PowerShell (kein `&&`):

```
$env:VIRTUAL_ENV = $null
$env:Path = "C:\Users\d4\AppData\Local\Programs\Python\Python312;" + $env:Path
. C:\Users\d4\esp\esp-idf\export.ps1
cd C:\Users\d4\Code\ESP32\esp32-puddledish-c3\firmware
idf.py build
```

Expected: Build OK, Target esp32s3, keine fehlenden Komponenten.

- [x] **Step 5: Flash + Monitor, visuell prüfen**

```
idf.py -p COM3 flash monitor
```

Expected Serial: `FluidBox layer 1` und `466x466 CO5300 ready`.

Expected Glas: rundes Bild, Kreuz durch die Mitte, Ring nah am Rand, kein seitlicher Versatz, kein toter Streifen unten (die letzten 4 Zeilen gehören zum Muster).

Wenn schwarz: nicht weiter zu Task 2. Pins/Gap/Init prüfen, nicht die Physik.

**Stopp.** User muss das Muster auf dem Glas bestätigt haben.

- [ ] **Step 6: Commit** — überspringen (kein Git).

---

### Task 2: Tote Perlen (Schicht 2)

**Files:**
- Create: `esp32-puddledish-c3/firmware/main/sim.h` (nur View-Typen + Snapshot-API)
- Create: `esp32-puddledish-c3/firmware/main/lattice.c`
- Create: `esp32-puddledish-c3/firmware/main/render.h` (Kopie `repo/fluidbox/main/render.h`)
- Create: `esp32-puddledish-c3/firmware/main/render.c` (Kopie + Patches unten)
- Modify: `esp32-puddledish-c3/firmware/main/config.h` — Box + Render-Konstanten
- Modify: `esp32-puddledish-c3/firmware/main/display.h` — zweite Flush-Signatur für Band-Index **nicht** einführen; Renderer auf `y0, rows` umstellen
- Modify: `esp32-puddledish-c3/firmware/main/CMakeLists.txt` — `render.c` `lattice.c`
- Modify: `esp32-puddledish-c3/firmware/main/main.c` — `render_init` + `render_frame`, Muster raus

**Interfaces:**
- Consumes: `display_acquire_band`, `display_flush_band(y0, rows, buf)`
- Produces:
  - `typedef struct { float x, y, z, speed; } sim_particle_view_t`
  - `void sim_init(void)` — füllt stehendes Gitter
  - `int sim_snapshot(sim_particle_view_t *out, int max)`
  - `void render_init(void)`
  - `void render_frame(void)`
  - Noch kein `sim_step`

- [x] **Step 1: config.h um Box und Farbe erweitern**

An `config.h` anhängen, Werte aus `repo/fluidbox/main/config.h` außer Auflösung/Box:

```c
#define BOX_W ((float)LCD_H_RES)
#define BOX_H ((float)LCD_V_RES)
#define BOX_D 75.0f
#define BOX_CORNER_R (BOX_W * 0.5f)

#define PX_PER_METER 12677.0f
#define PX_PER_MM (PX_PER_METER / 1000.0f)
#define BOX_BACK_FILLET_MM 2.0f
#define BOX_BACK_FILLET (BOX_BACK_FILLET_MM * PX_PER_MM)
#define BOX_FRONT_FILLET (BOX_BACK_FILLET * 0.25f)

#define PARTICLE_MAX 1000
#define PARTICLE_COUNT 900
#define REST_SPACING 17.0f

#define PROJ_FOCAL 220.0f
#define PARTICLE_RADIUS_PX 6.5f
#define DISC_MAX_R 10
#define HIGHLIGHT_ENABLE 1
#define HIGHLIGHT_LIFT 0.55f
#define SPEED_LEVELS 64
#define DEPTH_LEVELS 16
#define SPEED_COLOR_MAX 5000.0f
#define SPEED_COLOR_GAMMA 0.55f
#define DEPTH_DIM_MIN 0.60f
```

`PX_PER_METER` bewusst Johannes lassen (nicht auf 266 ppi des 1.75er umrechnen) — gleiches Tempo wie der Tweet.

- [x] **Step 2: sim.h + stehendes Gitter**

`sim.h` aus `repo/fluidbox/main/sim.h` kopieren. In Layer 2 dürfen `sim_step` / `sim_reset` / `sim_stats` als leere Stubs in `lattice.c` stehen, damit der Header durchcompiliert.

`lattice.c`: `sim_init` legt 900 Punkte auf ein Gitter in der unteren Hälfte des **Kreises** (nicht ins Rechteck bis in die Ecken, die es nicht gibt):

```c
/* Für i in 0..PARTICLE_COUNT-1: Schicht von unten, x/z wie Johannes,
   Punkte mit hypot(x-cx, y-cy) > BOX_CORNER_R - 8 überspringen und
   den nächsten Gitterplatz nehmen, bis 900 innen liegen. */
/* speed staffeln: 0 in der Tiefe, höher vorne — nur damit die Rampe sichtbar ist. */
/* z von ~10 bis ~65 über die Schichten. */
```

`sim_snapshot` kopiert das statische Array. Kein Mutex nötig in Layer 2.

- [x] **Step 3: render.c kopieren und an variable Streifen anpassen**

Kopieren: `repo/fluidbox/main/render.c` → `firmware/main/render.c` (und `render.h`).

Patches, nicht das ganze File neu schreiben:

1. `display_flush_band(band, buf)` ersetzen durch Zeilenhöhe:

```c
const int band_y0 = band * BAND_ROWS;
int rows = BAND_ROWS;
if (band_y0 + rows > LCD_V_RES) {
    rows = LCD_V_RES - band_y0;
}
/* memset nur rows * LCD_H_RES, nicht BAND_PIXELS wenn letzter Streifen kürzer */
memset(buf, 0, (size_t)LCD_H_RES * (size_t)rows * sizeof(uint16_t));
/* draw_disc vertikal auf band_y0+rows clippen, nicht hart BAND_ROWS */
display_flush_band(band_y0, rows, buf);
```

2. `draw_disc`: statt `band_y0 + BAND_ROWS` die tatsächliche `rows` nutzen. Signatur um `int rows` erweitern.

3. `SWAP16` / LUT **behalten** (Orb swapt ebenfalls).

4. `BAND_COUNT` kommt aus `config.h` (ceil), nicht `LCD_V_RES / BAND_ROWS`.

- [x] **Step 4: main.c auf Renderer umstellen**

```c
void app_main(void)
{
    ESP_LOGI(TAG, "FluidBox layer 2");
    ESP_ERROR_CHECK(display_init());
    sim_init();
    render_init();
    for (;;) {
        render_frame();
        vTaskDelay(1);
    }
}
```

CMake `SRCS` um `render.c` und `lattice.c` erweitern.

- [x] **Step 5: Build, Flash, Glas**

Dieselbe IDF-Export-Kette wie Task 1, dann `idf.py -p COM3 flash monitor`.

Expected Glas: viele blaue Perlen, Mitte voll, außerhalb des Kreises schwarz, hinten kleiner und dunkler, nichts bewegt sich.

Expected Serial: `FluidBox layer 2`, `466x466 CO5300 ready`, Seed-Log mit 900.

**Stopp.** User bestätigt stehende Perlen.

- [ ] **Step 6: Commit** — überspringen.

---

### Task 3: Solver, Gerät liegt (Schicht 3)

**Files:**
- Create: `esp32-puddledish-c3/firmware/main/sim.c` (Kopie `repo/fluidbox/main/sim.c`)
- Delete: `esp32-puddledish-c3/firmware/main/lattice.c` (durch echten Solver ersetzen)
- Modify: `esp32-puddledish-c3/firmware/main/sim.h` — volle API aus dem Repo
- Modify: `esp32-puddledish-c3/firmware/main/config.h` — restliche Sim-Konstanten aus dem Repo
- Modify: `esp32-puddledish-c3/firmware/main/sim.c` — `GRID_CX`/`GRID_CY`
- Modify: `esp32-puddledish-c3/firmware/main/CMakeLists.txt` — `sim.c` statt `lattice.c`, `-O2 -ffast-math` wie Repo
- Modify: `esp32-puddledish-c3/firmware/main/main.c` — zwei Tasks, feste Schwerkraft, kein IMU

**Interfaces:**
- Consumes: `render_frame`, Display
- Produces:
  - `void sim_init(void)`
  - `void sim_reset(void)`
  - `void sim_step(float dt_real, const sim_forces_t *forces)`
  - `int sim_snapshot(sim_particle_view_t *out, int max)`
  - `void sim_stats(sim_stats_t *out)`
  - Sim-Task Core 1, Render-Task Core 0

- [ ] **Step 1: Sim-Konstanten nach config.h**

Aus `repo/fluidbox/main/config.h` übernehmen (unverändert lassen, nicht „verbessern“):

`TIME_SCALE 0.100f`, `SIM_DT_MAX 0.0022f`, `GRAVITY_MPS2 9.81f`, `GRAVITY_GAIN 2.2f`, `SMOOTH_RADIUS 28.0f`, `SUBSTEPS 1`, `K_PRESSURE 400000.0f`, `K_NEAR_PRESSURE 800000.0f`, `MAX_DISPLACEMENT 4.0f`, `WALL_JITTER 0.35f`, `VISC_SIGMA 45.0f`, `VISC_BETA 0.03f`, `WALL_RESTITUTION 0.25f`, `WALL_FRICTION 0.96f`.

IMU-Macros dürfen schon in der Header-Datei stehen, werden in Task 3 nicht gelesen.

- [ ] **Step 2: sim.c kopieren, Gitter aufs Quadrat setzen**

Kopie `repo/fluidbox/main/sim.c` → `firmware/main/sim.c`.

Nur diese drei Defines ändern:

```c
#define GRID_CX 16
#define GRID_CY 16
#define GRID_CZ 2
```

Begründung: Zellen müssen ≥ `SMOOTH_RADIUS` (28) sein. `466/16 = 29.125`, `75/2 = 37.5`. Johannes’ 13×16 war für 368×448.

`resolve_walls` nicht anfassen. Mit `BOX_CORNER_R = BOX_W/2` wird das Rechteck von allein zum Kreis.

`sim_reset` füllt weiter ein Rechteck-Gitter; Teilchen außerhalb des Kreises schiebt `resolve_walls` in der ersten Step-Runde nach innen. Nach `sim_reset` einmal `resolve_walls` nicht extra exportieren — der erste `sim_step` reicht. Optional in `sim_reset` am Ende die existierende `resolve_walls` aufrufen, falls sie `static` bleibt: dann nach dem Seed-Loop `resolve_walls();` ergänzen (Funktion liegt in derselben Datei). **Ja, das tun** — sonst zeigt Frame 0 noch Ecken-Punkte außerhalb des Glases.

CMake wie Repo:

```cmake
idf_component_register(
    SRCS "main.c" "display.c" "render.c" "sim.c"
    INCLUDE_DIRS "."
)
target_compile_options(${COMPONENT_LIB} PRIVATE -O2 -ffast-math)
```

`lattice.c` aus SRCS nehmen und die Datei löschen.

- [ ] **Step 3: Zwei Tasks, Gravitation fest nach unten**

`main.c` an Johannes anlehnen, **ohne** `imu_init` / `button_init`:

```c
static void render_task(void *arg)
{
    (void)arg;
    for (;;) {
        render_frame();
        s_frames++;
        vTaskDelay(1);
    }
}

static void sim_task(void *arg)
{
    (void)arg;
    sim_forces_t forces = {
        .gravity = {0.0f, GRAVITY_GAIN * GRAVITY_MPS2 * PX_PER_METER, 0.0f},
        .omega = {0, 0, 0},
        .alpha = {0, 0, 0},
    };
    int64_t last_us = esp_timer_get_time();
    for (;;) {
        const int64_t now = esp_timer_get_time();
        float dt = (float)(now - last_us) * 1e-6f;
        last_us = now;
        if (dt > 0.05f) dt = 0.05f;
        else if (dt < 1e-4f) dt = 1e-4f;
        sim_step(dt, &forces);
        s_steps++;
        vTaskDelay(1);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "FluidBox layer 3");
    ESP_ERROR_CHECK(display_init());
    sim_init();
    render_init();
    xTaskCreatePinnedToCore(sim_task, "sim", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(render_task, "render", 4096, NULL, 5, NULL, 0);
    /* stats_loop wie repo/main.c, ohne imu_raw_accel */
}
```

Stats-Log wie Johannes, Feld `accel` weglassen oder `0 0 0`. `rho` muss nach ein paar Sekunden nahe `rest_density` (~1.17) liegen.

- [ ] **Step 4: Build, Flash, Glas + Serial**

Expected Glas: Perlen sacken nach unten (Bild-unten = +y), Klumpen bleibt zusammen, Oberfläche beruhigt sich, Kreisrand hält.

Expected Serial: `steps/s` grob 30–40, `rho` ~1.17/1.17, `speed avg` nicht dauernd >200 im Liegen. `grid cell … smaller than radius` darf **nicht** erscheinen.

Wenn es köchelt: nicht GRID ändern. Erst Log zeigen, dann ggf. `TIME_SCALE` — nur mit User-OK.

**Stopp.** User bestätigt sackende Flüssigkeit.

- [ ] **Step 5: Commit** — überspringen.

---

### Task 4: IMU + PWR (Schicht 4)

**Files:**
- Create: `esp32-puddledish-c3/firmware/main/imu.c` / `imu.h` (Kopie Repo)
- Create: `esp32-puddledish-c3/firmware/main/button.c` / `button.h` (Kopie Repo)
- Modify: `esp32-puddledish-c3/firmware/main/idf_component.yml` — `waveshare/qmi8658: "^2.0.0"`
- Modify: `esp32-puddledish-c3/firmware/main/config.h` — IMU-Maps aus dem Repo als Start
- Modify: `esp32-puddledish-c3/firmware/main/CMakeLists.txt` — `imu.c` `button.c`
- Modify: `esp32-puddledish-c3/firmware/main/main.c` — I2C 15/14, `imu_read`, PWR → `sim_reset`
- Modify: `johannes-fluidbox/QUELLE.md` — nur Hinweis auf `esp32-puddledish-c3/` / `esp32-puddledish-rust/`, Clone nicht überschreiben

**Interfaces:**
- Consumes: I2C-Bus, `sim_step`, `sim_reset`
- Produces:
  - `esp_err_t imu_init(i2c_master_bus_handle_t bus)`
  - `bool imu_read(float dt, sim_forces_t *out)`
  - `void imu_raw_accel(float out[3])`
  - `esp_err_t button_init(i2c_master_bus_handle_t bus)`
  - `bool button_take_short_press(void)`
  - Start-Maps (können falsch sein):

```c
#define IMU_MAP_X(ax, ay, az) (ay)
#define IMU_MAP_Y(ax, ay, az) (-(ax))
#define IMU_MAP_Z(ax, ay, az) (az)
```

- [ ] **Step 1: imu + button kopieren, I2C in main**

Dateien 1:1 aus `repo/fluidbox/main/`. I2C wie Repo und Orb: SDA 15, SCL 14, `I2C_NUM_0`.

`imu_init` / `button_init` Fehler = Log + weiter, nicht `ESP_ERROR_CHECK` (Flüssigkeit fällt dann weiter fest nach unten).

`sim_task`: `imu_read(dt, &forces)` vor `sim_step`. Alle 25 ms `button_take_short_press()` → `sim_reset()` + Log `PWR pressed, resetting fluid`.

`button.c` nur EXIO4 als Input setzen, andere Bits des TCA9554 nicht überschreiben (steht schon so im Repo — nicht „aufräumen“).

- [ ] **Step 2: Build, Flash, Achsen prüfen**

Serial `accel` bei flach, Glas oben: Betrag ~9.8. Drei Lagen notieren:

| Lage | erwartet (Johannes, nur Orientierung) |
|---|---|
| flach, Glas oben | z dominant |
| aufrecht, USB rechts | y oder x dominant |
| rechte Kante unten | die dritte Achse |

Glas: neigen gießt zur tiefen Kante. Schütteln gibt helle/weiße Spritzer. PWR kurz = neuer Klumpen. PWR lang (~6 s) = aus. BOOT unberührt; `idf.py flash` ohne BOOT-Tanz muss weiter gehen.

Wenn die Flüssigkeit zur Decke läuft: **ein** Vorzeichen in `IMU_MAP_*` kippen, neu flashen. Kein Umbau von `imu.c`.

- [ ] **Step 3: DESIGN.md / QUELLE.md auf „v1 auf dem Glas“ setzen, sobald User ok sagt**

Nicht vorher „fertig“ schreiben.

- [ ] **Step 4: Commit** — überspringen.

---

## Self-review (gegen DESIGN.md)

| Spec | Task |
|---|---|
| C-Port, `firmware/`, `repo/` unangetastet | Task 1–4 |
| Display vom Orb, 466, Gap 6, PCLK 38 | Task 1 |
| Streifen, letzter kürzer (21×22+4) | Task 1 + Render-Patch Task 2 |
| Tote Perlen, Kreis, Tiefe | Task 2 |
| Solver, rho, Gerät liegt | Task 3 |
| IMU + PWR, BOOT weg | Task 4 |
| 900 / BOX_D 75 / Zylinder über Corner-R | Task 2–3 |
| Kein Touch/Audio/Rust | nirgends angelegt |
| Falsche IMU-Achse = Vorzeichen | Task 4 Step 2 |

Keine TBDs. Kein „ähnlich wie Task N“ ohne den Code. Git bewusst ausgelassen.
