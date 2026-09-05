# ESP32 Cubli (V1)

A 3D self-balancing reaction wheel inverted pendulum (Cubli) based on the ESP32 and SimpleFOC. This repository contains the V1 firmware for balancing on edges using a Linear Quadratic Regulator (LQR) control scheme.

**Project Status: V1 Sunshined / V2 in Planning** > The V1 software architecture (State Estimation, FOC, and LQR) is fully operational and mathematically validated. Physical balancing is currently limited by the $V_{max}$ and $K_v$ ratings of the V1 gimbal motors, leading to actuator saturation before sufficient angular momentum can be transferred. Development is pivoting to V2 hardware specifications.

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

## V1 Hardware Post-Mortem & V2 Roadmap

Extensive hardware-in-the-loop testing and MATLAB kinematic modeling revealed that the V1 physical actuators are mathematically undersized for the chassis inertia.

**The Momentum Limit**
The system utilizes 90 KV motors powered by an 11.1V 3S LiPo, placing the absolute theoretical speed ceiling at ~1000 RPM. When the cube deviates from the $0.00$ rad balance point, the required corrective torque ($\tau_{cmd} = -Kx$) instantly demands a voltage exceeding the battery's physical limit. The reaction wheels cannot accelerate fast enough ($di/dt$) to generate the necessary counter-torque before the chassis falls past the recoverable $5^\circ$ threshold.

**Driver Saturation**
Because the LQR controller saturates attempting to overcome gravity, the high-frequency switching causes massive inductive flyback spikes from the motor coils. These spikes exceeded the breakdown voltage of the DRV8313 MOSFETs, leading to hardware failure during edge-drop testing.

**V2 Hardware Upgrades**
The V1 C++ LQR architecture and custom ground-frame complementary filter will be ported directly to V2. Hardware upgrades will include:
* **Higher Torque Motors:** Lower KV rating with larger stators for higher peak impulse.
* **Increased Voltage:** Transition to a 4S (14.8V) or 6S (22.2V) power system to drastically raise the RPM ceiling and increase the total angular momentum capacity ($L = I\omega$).
* **Robust Motor Drivers:** Implementation of drivers with higher peak current ratings and dedicated transient voltage suppression (TVS) to safely handle high-frequency LQR commutation and flyback.
* **Dual-Core Processing:** Migrating the State Estimation matrix math to ESP32 Core 0, reserving Core 1 exclusively for the 3-phase FOC commutation and LQR calculation to maximize $dt$ resolution.

**Toolchain & Environment Philosophy**
V1 was built on PlatformIO with the Arduino framework, which abstracted away the ESP32's toolchain, build system, and peripheral communication (I2C, PWM, etc.). While the control logic (state estimation, FOC, LQR) held up, the surrounding environment was patchy and left the actual hardware interaction as a black box. V2 will move away from PlatformIO and Arduino-style abstraction layers in favor of self-handling the ESP32 toolchain, build process, and low-level peripheral drivers directly. This is a deliberate step toward learning embedded systems development at a lower level, rather than a strict technical requirement.

## Repository Layout

```text
📂 v1-firmware   # ESP32 / PlatformIO C++ firmware (state estimation, FOC, LQR control loop)
📂 v1-modeling   # MATLAB scripts deriving the LQR gains used by the firmware
```

See each subproject's README for build instructions, pinout, dependency versions, and file structure: [`v1-firmware/README.md`](v1-firmware/README.md), [`v1-modeling/README.md`](v1-modeling/README.md).
