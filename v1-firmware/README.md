# V1 Firmware

ESP32 firmware implementing state estimation, FOC motor commutation, and the LQR balance controller. See the [top-level README](../README.md) for system architecture and project status.

## Build

Built with **PlatformIO** (`framework = arduino`, `board = esp32dev`). Open this directory as a PlatformIO project and build/upload normally; dependencies in `platformio.ini` are pulled automatically.

## Pinout & Configuration (`include/Config.h`)

| Component | ESP32 Pin / Channel | Notes |
| :--- | :--- | :--- |
| **I2C Bus** | SDA: 23, SCL: 22 | Requires pull-up resistors |
| **Driver Enable** | EN: 33 | Global Enable for all 3 drivers |
| **Motor X (0)** | IN: 16, 15, 2 | Mux Channel 3 *(Note: GPIO 2 is strapped to boot mode)* |
| **Motor Y (1)** | IN: 25, 26, 27 | Mux Channel 2 |
| **Motor Z (2)** | IN: 18, 17, 5 | Mux Channel 6 |

## Software Dependencies

Pinned in `platformio.ini`:

1. `askuric/Simple FOC @ 2.3.2` (pinned exactly — kept in sync with the ESP32 Arduino framework version in use)
2. `tomstewart89/BasicLinearAlgebra @ ^5.1.0`
3. `rfetick/MPU6050_light @ ^1.1.0`

## File Structure

```text
📂 src
 ├── main.cpp         # Main execution loop and setup calls
 └── Cubli.cpp        # Class implementation, math, and LQR loops
📂 include
 ├── Cubli.h          # Class definition and data structures
 ├── Config.h         # System constants, pinouts, and hardware specs
 └── gains.h          # Pre-calculated LQR gain matrices, derived in ../v1-modeling
```

`include/gains.h` is hand-transcribed from the `Ke_max`/`Ke_min` gain matrices computed by `../v1-modeling/main.m` — see that project's README for the derivation.
