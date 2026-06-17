/**
 * @file main.c
 * @brief Dual Asynchronous UART Bridge for SIM808 Telematics Module.
 * @details Implements a bare-metal register-level configuration for USART1 and USART2.
 * Operates as an asynchronous pass-through bridge between a PC terminal and an external 
 * GSM/GPS module, utilizing interrupt-driven linear buffering and AT command parsing.
 * @author Daniel Ruiz Perez
 * @date 2026-06-17
 */

#include "main.h"
#include "stdio.h"
#include "string.h"

#define MAX_BUFFER_SIZE 560 

// --- Asynchronous Buffers & State Flags ---
volatile char buffer_pc[MAX_BUFFER_SIZE];
volatile uint16_t idx_pc = 0;
volatile uint8_t pc_ready = 0;

volatile char buffer_sim[MAX_BUFFER_SIZE];
volatile uint16_t idx_sim = 0;
volatile uint8_t sim_ready = 0; 

/**
 * @brief Initializes USART1 (SIM808) and USART2 (PC Terminal) at the register level.
 */
void init_uarts(void) {
    // 1. Enable Peripheral Clocks
    RCC->AHB1ENR |= (1 << 0);       // GPIOA Clock
    RCC->APB2ENR |= (1 << 4);       // USART1 Clock (APB2 Bus)
    RCC->APB1ENR |= (1 << 17);      // USART2 Clock (APB1 Bus)

    // 2. Configure GPIO Pins (PA2, PA3 for USART2 | PA9, PA10 for USART1)
    GPIOA->MODER &= ~((3 << 4) | (3 << 6) | (3 << 18) | (3 << 20));
    GPIOA->MODER |=  ((2 << 4) | (2 << 6) | (2 << 18) | (2 << 20)); // Alternate Function Mode

    // 3. Map Alternate Functions (AF7 for USART1 & USART2)
    GPIOA->AFR[0] &= ~((15 << 8) | (15 << 12));
    GPIOA->AFR[0] |=   ((7 << 8) | (7 << 12));   // AF7 for PA2, PA3
    GPIOA->AFR[1] &= ~((15 << 4) | (15 << 8));
    GPIOA->AFR[1] |=   ((7 << 4) | (7 << 8));    // AF7 for PA9, PA10

    // 4. Configure Baud Rate (9600 bps @ 16 MHz System Clock)
    // Mantissa = 104, Fraction = 3 (16MHz / (16 * 9600) = 104.166 -> 0.166 * 16 = 2.6 ~ 3)
    USART1->BRR = (104 << 4) | 3;
    USART2->BRR = (104 << 4) | 3;

    // 5. Enable Transmitters, Receivers, and RX Interrupts
    USART1->CR1 = (1 << 13) | (1 << 5) | (1 << 3) | (1 << 2);
    USART2->CR1 = (1 << 13) | (1 << 5) | (1 << 3) | (1 << 2);

    // 6. Enable Interrupt Vectors in NVIC
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(USART2_IRQn);
}

// --- USART Transmit Primitives (Blocking) ---

void usart1_putc(char c) {
    while (!(USART1->SR & (1 << 7))); // Wait for TXE (Transmit Data Register Empty)
    USART1->DR = c;
}

void usart1_puts(char *s) {
    while(*s) usart1_putc(*s++);
}

void usart2_putc(char c) {
    while (!(USART2->SR & (1 << 7)));
    USART2->DR = c;
}

void usart2_puts(char *s) {
    while(*s) usart2_putc(*s++);
}

// --- Interrupt Service Routines (ISRs) ---

/**
 * @brief Handles incoming bytes from the SIM808 module.
 * @details Forwards characters instantly to the PC and parses full lines into a buffer.
 */
void USART1_IRQHandler(void) {
    if (USART1->SR & (1 << 5)) { // RXNE flag (Read Data Register Not Empty)
        char c = USART1->DR;
        usart2_putc(c); // Direct Hardware Echo

        // Asynchronous Payload Capture
        if (!sim_ready) {
            if (c == '\r' || c == '\n') {
                if (idx_sim > 0) {
                    buffer_sim[idx_sim] = '\0';
                    sim_ready = 1;
                }
            } else if (idx_sim < MAX_BUFFER_SIZE - 1) {
                buffer_sim[idx_sim++] = c;
            }
        }
    }
}

/**
 * @brief Handles incoming bytes from the PC Terminal.
 */
void USART2_IRQHandler(void) {
    if (USART2->SR & (1 << 5)) {
        char c = USART2->DR;

        if (c == '\r' || c == '\n') {
            buffer_pc[idx_pc] = '\0';
            if (idx_pc > 0) pc_ready = 1;
            idx_pc = 0; // Reset index for next command
        } else if (idx_pc < MAX_BUFFER_SIZE - 1) {
            buffer_pc[idx_pc++] = c;
        }
    }
}

/**
 * @brief Performs a handshake synchronization with the SIM808 module.
 */
void sincronizar_sim808(void) {
    uint8_t conectado = 0;
    uint8_t intentos = 0;

    usart2_puts("\r\nIniciando secuencia de sincronizacion...\r\n");

    while (!conectado) {
        sim_ready = 0;
        idx_sim = 0;
        memset((void*)buffer_sim, 0, MAX_BUFFER_SIZE);

        usart1_puts("AT\r\n");
        usart2_puts("Enviando: AT... ");

        HAL_Delay(500); // Allow hardware to process and respond

        if (sim_ready) {
            if (strstr((char*)buffer_sim, "OK") != NULL) {
                usart2_puts("Modulo Detectado!\r\n");
                conectado = 1;
            } else {
                usart2_puts("Respuesta inesperada.\r\n");
            }
        } else {
            usart2_puts("Sin respuesta.\r\n");
        }

        intentos++;
        if (intentos > 10 && !conectado) {
            usart2_puts("ALERTA: El modulo no responde. Revisa voltaje o cableado.\r\n");
            intentos = 0; 
        }

        if (!conectado) HAL_Delay(500);
    }
}

void SystemClock_Config(void);

int main(void){
  HAL_Init();
  SystemClock_Config();
  init_uarts();
  sincronizar_sim808();

  usart2_puts("\r\n--- Puente STM32-SIM808 a 9600 bps ---\r\n");

  while (1) {
      // Main Loop Command Dispatcher
      if (pc_ready) {
          if (strncmp((char*)buffer_pc, "AT", 2) == 0) {
              sim_ready = 0;
              idx_sim = 0;
              usart1_puts((char*)buffer_pc);
              usart1_puts("\r\n");
          }
          pc_ready = 0;
          idx_pc = 0;
      }
  }
}
