# STM32F4 Bare-Metal Dual UART Bridge

## Project Overview
This firmware demonstrates bare-metal serial communication on the ARM Cortex-M4 architecture (STM32F407VGT6). It acts as an asynchronous pass-through bridge between a PC diagnostic terminal and an external SIM808 GSM/GPS module. It bypasses HAL initialization structures for peripheral configuration, utilizing direct register manipulation to establish precise baud rates and interrupt-driven buffering for telematics command parsing.

## Hardware Architecture & Configuration
* Microcontroller: STM32F407VGT6 (ARM Cortex-M4)
* Clock Management: Manual enablement of the AHB1 bus for GPIO routing, alongside APB1 and APB2 buses to provide independent clock signals to the USART2 and USART1 peripherals.
* Port Configuration: 
  * Alternate Functions: GPIOA.2 / GPIOA.3 (USART2) and GPIOA.9 / GPIOA.10 (USART1) are explicitly configured to Alternate Function 7 (AF7) to route the hardware serial transmission lines.
*USART Configuration: Manual calculation and assignment of the BRR (Baud Rate Register) mantissa and fraction bits to establish a strict 9600 bps connection utilizing the 16MHz system clock.

## Key Technical Features
1. Asynchronous ISR Buffering: Implements non-blocking USARTx_IRQHandler vectors to capture incoming byte streams into linear volatile memory buffers, explicitly preventing CPU stalling or buffer overruns during high-latency cellular network responses.
2. String Parsing & Synchronization: Utilizes standard C string manipulation (`strstr, strncmp`) within a dedicated handshake routine to actively ping the hardware with `AT` commands and dynamically validate the `OK` acknowledgment before allowing main-loop execution.
3. Direct Hardware Echo: Configures the Interrupt Service Routines to instantly echo received byte payloads from the GSM/GPS module directly to the PC terminal, ensuring zero-latency telemetry monitoring.
