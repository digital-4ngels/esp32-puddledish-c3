# Puddle Dish — C-Port 1.75-B

Unser ESP-IDF-C-Port auf Waveshare **ESP32-S3-Touch-AMOLED-1.75-B** (466×466).

Johannes’ Original bleibt unberührt in `../johannes-fluidbox/repo/`. Nicht 1:1 flashen.

```
. C:\Users\d4\esp\esp-idf\export.ps1
cd C:\Users\d4\Code\ESP32\esp32-puddledish-c3\firmware
idf.py -p COM3 flash monitor
```

Spec: `DESIGN.md`. Rust-Port: `../esp32-puddledish-rust/`.
