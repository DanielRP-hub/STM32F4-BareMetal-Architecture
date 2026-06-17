# Hardware-in-the-Loop (HIL) Motor Control & Telemetry Dashboard

## Project Overview
This repository represents a full-stack embedded engineering solution combining bare-metal C real-time firmware with a Python-based Supervisory Control and Data Acquisition (SCADA) GUI. It demonstrates the ability to implement advanced Digital Control Theory (PID and Difference Equation Compensators) on an STM32 microcontroller, coupled with an asynchronous Python dashboard for dynamic tuning and HIL (Hardware-in-the-Loop) validation.

## Architecture Architecture

### 1. STM32 Firmware (Bare-Metal & CMSIS)
* **Deterministic Execution:** Utilizes `TIM6` hardware interrupts to enforce a strict, hard-real-time 20ms (50Hz) execution loop. This guarantees exact $\Delta t$ sampling periods required for accurate mathematical integration and derivation in the control models.
* **Hardware Quadrature Decoding:** Bypasses software interrupts by configuring `TIM3` in Hardware Encoder Mode. The silicon edge-detector processes high-RPM multi-turn tracking with zero CPU overhead.
* **Non-Blocking Telemetry:** Implements a custom UART Ring Buffer (Circular Buffer) triggered by TXE/RXNE hardware interrupts. This decouples the 115200 bps serial communication from the main loop, ensuring the CPU never stalls while parsing commands or broadcasting telemetry.
* **Digital Signal Processing:** * Implementation of Low-Pass Moving Average filters to clean raw sensor noise.
  * Anti-Windup integral clamping to protect the system during extreme actuator saturation.
  * Z-transform Lead-Lag Compensators translated into C-based difference equations storing historical sample states ($z^{-1}$, $z^{-2}$).

### 2. Python HIL Dashboard (CustomTkinter & Matplotlib)
* **Asynchronous Threading:** Uses Python's `threading` library to isolate the serial port listener from the UI event loop. This prevents the high-frequency 50Hz incoming telemetry stream from freezing the GUI.
* **Real-Time Oscilloscope:** Leverages `matplotlib` with deque buffers to render zero-latency dynamic line charts comparing kinematic setpoints versus real hardware responses.
* **Dynamic Parameter Injection:** Allows the user to construct `<START,MODE,ID,K1,K2,K3,K4,K5>` protocol vectors via the UI, modifying the STM32's control algorithm logic and operational mode (Speed vs Position) on the fly without halting execution.
