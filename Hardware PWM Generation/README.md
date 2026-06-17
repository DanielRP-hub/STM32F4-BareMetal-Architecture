## Project Overview
This firmware demonstrates bare-metal serial communication on the ARM Cortex-M4 architecture (STM32F407VGT6). This module demonstrates the direct configuration of the STM32F4's Advanced-Control Timers (TIM4) to generate deterministic, high-frequency Pulse Width Modulation (PWM) signals entirely via hardware, freeing the CPU from bit-banging tasks.

* **Technical Implementation:**
    * **Alternate Function Multiplexing:** Direct manipulation of the `GPIOD->AFR[1]` (Alternate Function High) register to explicitly route TIM4 output channels 1-4 to physical pins PD12 through PD15 using the AF2 bitmask.
    * **Timebase Configuration:** Bare-metal calculation and assignment of the Prescaler (`PSC`) and Auto-Reload Register (`ARR`) to establish a precise 1kHz hardware timebase from the 16MHz system clock.
    * **Capture/Compare Execution:** Configuring `CCMR1` and `CCMR2` registers to activate PWM Mode 1, and enabling outputs via the `CCER` register. 
    * **Dynamic Duty Cycle Scaling:** Dynamically updating the `CCRx` shadow registers in the main thread to alter the duty cycle seamlessly without interrupting the hardware timer execution.
