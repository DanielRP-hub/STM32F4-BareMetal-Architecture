# STM32F4 External Interrupts (EXTI) Implementation

## Project Overview
This project focuses on the configuration of the STM32F407VGT6's External Interrupt (EXTI) subsystem at the register level. It demonstrates how to bypass high-level HAL abstractions to directly manage peripheral clocks, pin modes, and the Nested Vectored Interrupt Controller (NVIC).

## Hardware Architecture
* Microcontroller: STM32F407VGT6 (ARM Cortex-M4)
* Input Logic: GPIO B and E configured as digital inputs with software-controlled pull-up/pull-down resistors.
* Trigger Logic: Configuration of Rising and Falling edge triggers for asynchronous response to physical button presses.
* Interrupt Handling: Implementation of custom `IRQHandler` vectors to manage multi-source hardware events.

## Technical Execution
1. SYSCFG Configuration: Correctly mapping EXTI lines to specific GPIO ports using the `SYSCFG->EXTICR` registers, a critical step often hidden by high-level drivers.
2. Bitwise Register Control: Precise clearing and setting of `MODER`, `PUPDR`, and `IMR` registers to ensure efficient memory usage and avoid accidental pin state corruption.
3. Edge Detection: Independent configuration of `RTSR` (Rising Trigger) and `FTSR` (Falling Trigger) bits to allow flexible interrupt activation based on switch state changes.
4. ISR Debouncing: Manual implementation of a volatile delay loop to mitigate mechanical switch bouncing, ensuring that each physical event is handled as a single logical trigger.
