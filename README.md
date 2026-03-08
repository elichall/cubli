# ESP32 Cubli (V1)

A 3D self-balancing reaction wheel inverted pendulum (Cubli) based on the ESP32 and SimpleFOC. This repository contains the V1 firmware for balancing on edges using a Linear Quadratic Regulator (LQR) control scheme.

## System Architecture

The control system decouples the hardware geometry from the balancing logic using coordinate transformations. The plant state is evaluated in the **Ground Frame** (aligned with gravity), while control efforts are mapped back to the **Natural Body Frame** (the physical motor axes).

### State Space Representation
The system evaluates a 12x1 state vector $\vec{x}$ encompassing the body frame and the three reaction wheels. For each axis $i \in \{X, Y, Z\}$, the state variables are:
* $\theta_i$: Body angle relative to gravity (rad)
* $\dot{\theta}_i$: Body angular velocity (rad/s)
* $\phi_i$: Reaction wheel angle (rad)
* $\dot{\phi}_i$: Reaction wheel angular velocity (rad/s)

Control input $\vec{u}$ is calculated via the optimal gain matrix $K$: 
$$\vec{u} = -K\vec{x}$$

### Coordinate Transformations
1. **Natural Body Frame:** The physical orthogonal axes of the cube where the motors reside.
2. **Virtual Body Frame:** An intermediate alignment frame.
3. **Ground Frame:** The active balancing frame where the Z-axis aligns with the gravity vector $\vec{g}$.

## Hardware Dependencies

* **Microcontroller:** ESP32 (DevKit V1)
* **IMU:** MPU6050 (I2C)
* **Encoders:** 3x AS5600 Magnetic Encoders (I2C)
* **Multiplexer:** TCA9548A (I2C, Address `0x70`)
* **Motor Drivers:** 3x DRV8313 / L6234 (3-PWM logic)
* **Motors:** 3x BLDC Gimbal Motors (7 Pole Pairs)
* **Power:** 3S LiPo Battery (11.1V Nominal)

## Pinout & Configuration (`Config.h`)

| Component | ESP32 Pin / Channel | Notes |
| :--- | :--- | :--- |
| **I2C Bus** | SDA: 23, SCL: 22 | Requires pull-up resistors |
| **Driver Enable** | EN: 33 | Global Enable for all 3 drivers |
| **Motor X (0)** | IN: 25, 26, 27 | Mux Channel 3 |
| **Motor Y (1)** | IN: 2, 5, 17 | Mux Channel 2 *(Note: GPIO 2 is strapped to boot mode)* |
| **Motor Z (2)** | IN: 16, 4, 15 | Mux Channel 6 |

## Software Dependencies

This project is built using **PlatformIO**. Ensure the following libraries are included in your `platformio.ini`:

1. `askuric/Simple FOC @ ^2.3.2`
2. `tomstewart89/BasicLinearAlgebra @ ^5.0.0`
3. `rfetick/MPU6050_light @ ^1.1.0`

## File Structure

```text
📂 src
 ├── main.cpp         # Main execution loop and setup calls
 └── Cubli.cpp        # Class implementation, math, and LQR loops
📂 include
 ├── Cubli.h          # Class definition and data structures
 ├── Config.h         # System constants, pinouts, and hardware specs
 └── gains.h          # Pre-calculated LQR gain matrices
