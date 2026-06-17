# STM32F4 Bare-Metal Register Mapping

## Project Overview
This firmware demonstrates true bare-metal programming on the ARM Cortex-M4 architecture (STM32F407VGT6). It entirely bypasses both the CMSIS device headers and the STM32 Hardware Abstraction Layer (HAL). Instead, it implements custom struct-based memory mapping to interact directly with the peripheral memory boundaries.

## Hardware Architecture & Configuration
* Microcontroller: STM32F407VGT6 (ARM Cortex-M4)
* Register Encapsulation: Custom `typedef struct` definitions exactly mirroring the silicon memory map for `GPIO` and `RCC` peripherals. Dummy memory padding ensures precise 32-bit register alignment.
* Clock Management: Manual enablement of the Advanced High-performance Bus 1 (AHB1) to route clock signals to specific GPIO ports without wasting power on unused domains.
* Port Configuration: 
  * Outputs: `GPIOA.0` through `GPIOA.5` are strictly configured as push-pull outputs to drive the logic sequence. Unused pins on this port remain in their reset configuration to prevent floating logic.
  * Inputs: `GPIOD.0` is configured as a standard digital input to read hardware button states.

## Key Technical Features
1. Memory-Mapped Pointers: Defines precise physical memory offsets (e.g., `AHB1_OFFSET 0x00020000UL`) and casts them to structured pointers (`RCC`, `GPIOA`), achieving O(1) memory access identical to professional vendor-supplied headers.
2. Bitwise Register Control: Modifies `MODER`, `OTYPER`, `IDR`, and `ODR` registers utilizing strict bit-masking to prevent overwriting adjacent pin configurations.
3. Compiler-Safe Debouncing: Implements a non-blocking logic flag (`boton_presionado`) combined with a `volatile` software delay variable to mitigate mechanical switch bounce, explicitly preventing GCC optimization algorithms (-O2/-O3) from stripping out the execution delay.
