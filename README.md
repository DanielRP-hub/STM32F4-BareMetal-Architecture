# STM32F4 Bare-Metal Architecture & Real-Time Systems

## Repository Overview
This repository serves as a comprehensive technical showcase of low-level firmware development on the STM32F407VGT6 (ARM Cortex-M4). The core objective is to bypass vendor-provided Hardware Abstraction Layers (HAL) to implement direct register manipulation, deterministic execution, and complex real-time asynchronous architectures. 

The progression of projects demonstrates a deep understanding of microcontroller memory maps, bus architectures (AHB/APB), peripheral multiplexing, and Digital Signal Processing (DSP) applied to motor control.

## Hardware Architecture
* **Microcontroller:** STM32F407VGT6 (ARM Cortex-M4 with FPU)
* **Clock Management:** Manual configuration of Reset and Clock Control (RCC) to optimize power consumption across isolated peripheral buses.
* **Interrupt Controller:** Advanced configuration of the Nested Vectored Interrupt Controller (NVIC) to manage asynchronous hardware events and prioritize critical execution routines.

---

## Projects Index

### 1. Bare-Metal Register Mapping & GPIO Control
Demonstrates foundational hardware control through custom `typedef struct` memory mapping, bypassing CMSIS/HAL headers for O(1) register access.
* **Focus:** `MODER`, `OTYPER`, and `ODR` register bitwise manipulation.
* **Technique:** Implementation of compiler-safe software debouncing using `volatile` variables to prevent GCC optimization (-O2/-O3) from stripping execution delays.

### 2. Asynchronous Interrupt-Driven HMI (EXTI)
Expands the architecture by introducing asynchronous hardware events, implementing a robust decoupling pattern between Interrupt Service Routines (ISRs) and main-thread application logic.
* **Focus:** Direct configuration of `SYSCFG->EXTICR` for interrupt mapping and multi-source ISR dispatching.
* **Technique:** `volatile` flag variables are used to signal state changes to a state-machine dispatcher within the main thread, keeping ISR execution times minimal and deterministic.

### 3. Dual Asynchronous UART Bridge (Telematics Gateway)
Acts as an asynchronous serial bridge between a PC terminal (USART2) and an external SIM808 GSM/GPS module (USART1), serving as a foundational layer for Telematics Control Units (TCU).
* **Focus:** Bare-metal calculation of the `USART->BRR` registers to establish precise baud rates.
* **Technique:** Implements non-blocking `USARTx_IRQHandler` vectors and Ring Buffers (Circular Buffers) to capture incoming byte streams, preventing CPU stalling during high-latency network responses.

### 4. Hardware PWM Generation (Advanced-Control Timers)
Demonstrates the direct configuration of Advanced-Control Timers (TIM4) to generate deterministic, high-frequency Pulse Width Modulation (PWM) signals entirely via hardware.
* **Focus:** Alternate Function (AF) multiplexing (`GPIOD->AFR[1]`) and Timebase Configuration (`PSC`, `ARR`).
* **Technique:** Dynamically updating `CCRx` shadow registers to alter duty cycles seamlessly without interrupting hardware timer execution, freeing the CPU from bit-banging tasks.

### 5. Digital Control & HIL Telemetry Dashboard
A full-stack embedded engineering solution combining bare-metal real-time firmware with a Python-based Supervisory Control and Data Acquisition (SCADA) GUI for Hardware-in-the-Loop (HIL) validation.
* **Focus:** Hard-real-time 20ms execution loop using `TIM6` and Hardware Quadrature Decoding using `TIM3`.
* **Technique:** Implementation of Discrete Lead-Lag Compensators and PID algorithms via difference equations. Features an asynchronous Python UI leveraging `matplotlib` and `threading` for zero-latency dynamic telemetry plotting and on-the-fly parameter injection.
