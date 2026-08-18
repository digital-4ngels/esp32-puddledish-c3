# Fluidbox 1.75-B — Design

Stand: 2026-08-17. Port von Johannes’ Fluidbox auf unser rundes Board.  
Spec zum Gegenlesen, noch kein Code.

## Ziel

Dasselbe Demo wie [der X-Post](https://x.com/JohannesTscharn/status/2085248949061922855), auf dem **1.75-B**:

- Blaue Perlen in einer flachen runden Dose (Glas = Display, hinten = Gehäuse)
- Neigen gießt, Schütteln spritzt hell/weiß
- PWR kurz = neu setzen, PWR lang = aus (Hardware), BOOT unberührt

Kein Touch, kein Mic, keine andere Farbe, kein Rust, nicht sein 1.8er-Binary.

## Board

Waveshare **ESP32-S3-Touch-AMOLED-1.75-B**, 466×466, COM3.  
Nicht das rechteckige 1.8er aus dem Repo.

## Stack

ESP-IDF-C-Projekt in `esp32-puddledish-c3/firmware/`.  
Johannes-Clone `../johannes-fluidbox/repo/` bleibt Referenz (MIT, nicht flashen). Orb-C und Orb-Rust bleiben unberührt.

| Datei | Herkunft |
|---|---|
| `display.c` | unser Orb (466, Gap x=6, PCLK 38, RST 39) |
| `sim.c` / `render.c` | Johannes |
| `imu.c` / `button.c` | Johannes, Achsen an diesem Board messen |
| `config.h` | neu: Kreis-Box, unsere Pins |

Zwei Tasks: Core 1 Physik, Core 0 Bild. Treffpunkt = kurze Kopie der 900 Positionen.

Kein Vollbild im RAM. Streifen à 22 Zeilen (letzter Streifen 4 Zeilen), wie der Orb.

## Form

Kreis auf dem Glas, flache Dose dahinter (`BOX_D = 75`, nicht die echte Gehäusetiefe — sonst Tunnel).  
Leichte Rundung an Glas und Rücken. 900 Partikel, gleicher Abstand wie bei Johannes (Volumen ist fast gleich).

Die Dose wird nicht gezeichnet. Man sieht nur, wo die Perlen abprallen.

## Bau in vier Schichten

Nächste Schicht erst, wenn die aktuelle auf dem Glas stimmt.

1. **Panel** — Ring + Kreuz. Kreis voll, kein Versatz.
2. **Tote Perlen** — Gitter steht still. Mitte voll, Rand leer, hinten kleiner/dunkler.
3. **Solver** — Gerät liegt, Perlen sacken und bleiben zusammen. Serial: `rho` ~1.17, nicht dauernd köcheln.
4. **IMU + PWR** — neigen/schütteln/reset. 1.75-B: `IMU_MAP_X=-ax Y=-ay Z=az` (User ok, hängend USB oben).

## Fertig (v1)

In der Hand: gießt, spritzt hell, PWR setzt zurück, fühlt sich flüssig an (kein Klumpen, kein Kochtopf).

**Stand 2026-08-17, User: „alles besser“** (halten/kippen).  
900 / `TIME_SCALE 0.068` / `GRAVITY_GAIN 1.8` / `SHAKE_GAIN 3.0` / r=7.2 / IMU `-ax,-ay,az`.

**Flach-Blick geparkt:** von oben auf dem Tisch wirkt die Pfütze langweilig. Später, nicht jetzt weiterdrehen.

## Risiko

Erstes IMU-Flash kann die Achsen verdrehen. Erwartet, ein Vorzeichen.

## Nicht in v1

Touch, Audio, mehr als 900 Teilchen, Host-Preview als Pflicht, Rust, OBD2-Code.
