/**
 * @file main.c
 * @brief External Interrupt (EXTI) configuration for STM32F407VGT6.
 * @details Configures GPIOs as inputs with pull-up/pull-down resistors, maps them 
 * to EXTI lines via SYSCFG, and implements interrupt service routines (ISRs) 
 * for button state management and LED toggling.
 * @author Daniel Ruiz Perez
 * @date 2024-06-25
 */

#include "main.h"

void SystemClock_Config(void);

/**
 * @brief Enables peripheral clocks for GPIO and SYSCFG.
 */
void init_clock(void){
    // Habilitar reloj para GPIOB, GPIOE y GPIOA (AHB1)
    RCC->AHB1ENR |= ((1 << 0) | (1 << 1) | (1 << 4));
    // Habilitar reloj para SYSCFG (APB2) para el direccionamiento de EXTI
    RCC->APB2ENR |= (1 << 14);
}

/**
 * @brief Configures input pins and external interrupt triggers.
 */
void init_exti(void){
    // 1. Configurar GPIOs como entradas
    GPIOB->MODER &= ~((1 << 3) | (1 << 2) | (1 << 5) | (1 << 4)); 
    GPIOE->MODER &= ~0xFFFFFFFF; // Set all E pins to Input

    // Configurar resistencias (Pull-up: 01, Pull-down: 10)
    GPIOB->PUPDR &= ~0xFFFFFFFF;
    GPIOB->PUPDR |= ((1 << 2) | (1 << 5)); // Pull-down en B1, B2

    GPIOE->PUPDR &= ~0xFFFFFFFF;
    GPIOE->PUPDR |= ((1 << 24) | (1 << 15) | (1 << 17) | (1 << 31));

    // 2. Mapeo de EXTI a GPIOs mediante SYSCFG
    SYSCFG->EXTICR[0] &= ~0xFFFF;
    SYSCFG->EXTICR[1] &= ~0xFFFF;
    SYSCFG->EXTICR[2] &= ~0xFFFF;
    SYSCFG->EXTICR[3] &= ~0xFFFF;

    SYSCFG->EXTICR[0] |= ((1 << 4) | (1 << 8));     // EXTI1 y EXTI2 en B
    SYSCFG->EXTICR[1] |= (1 << 14);               // EXTI7 en E
    SYSCFG->EXTICR[2] |= (1 << 2);                // EXTI8 en E
    SYSCFG->EXTICR[3] |= ((1 << 2) | (1 << 14));  // EXTI12 y EXTI15 en E

    // 3. Configurar detección de flancos
    EXTI->RTSR &= ~(0x7FFFFF);
    EXTI->FTSR &= ~(0x7FFFFF);

    EXTI->RTSR |= ((1 << 2) | (1 << 8) | (1 << 7) | (1 << 15));
    EXTI->FTSR |= ((1 << 1) | (1 << 12) | (1 << 7) | (1 << 15));

    // 4. Habilitar interrupciones y activar NVIC
    EXTI->IMR |= ((1 << 1) | (1 << 2) | (1 << 7) | (1 << 8) | (1 << 12) | (1 << 15));

    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_EnableIRQ(EXTI2_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
 * @brief Configures LED pins as push-pull outputs.
 */
void init_LED(void){
    GPIOA->MODER |= ((1 << 4) | (1 << 6) | (1 << 8) | (1 << 10) | (1 << 12) | (1 << 14));
}

// ---------------- Interrupt Service Routines (ISRs) ----------------

void EXTI1_IRQHandler(void){
    if (EXTI->PR & (1 << 1)){
        for(volatile int i = 0; i < 20000; i++); // Debounce manual
        if (!(GPIOB->IDR & (1 << 1))) {
           GPIOA->ODR ^= (1 << 2);
        }
        EXTI->PR |= (1 << 1); // Clear pending bit
    }
}

void EXTI2_IRQHandler(void){
    if (EXTI->PR & (1 << 2)){
        for(volatile int i = 0; i < 20000; i++);
        if (GPIOB->IDR & (1 << 2)) {
            GPIOA->ODR ^= (1 << 3);
        }
        EXTI->PR |= (1 << 2);
    }
}

void EXTI9_5_IRQHandler(void){
    if (EXTI->PR & (1 << 7)){
        if (GPIOE->IDR & (1 << 7)) {
            GPIOA->ODR |= (1 << 4);
        } else {
            GPIOA->ODR &= ~(1 << 4);
        }
        EXTI->PR |= (1 << 7);
    }

    if (EXTI->PR & (1 << 8)){
        for(volatile int i = 0; i < 20000; i++);
        if (GPIOE->IDR & (1 << 8)) {
            GPIOA->ODR ^= (1 << 5);
        }
        EXTI->PR |= (1 << 8);
    }
}

void EXTI15_10_IRQHandler(void){
    if (EXTI->PR & (1 << 12)){
        for(volatile int i = 0; i < 20000; i++);
        if (!(GPIOE->IDR & (1 << 12))) {
            GPIOA->ODR ^= (1 << 6);
        }
        EXTI->PR |= (1 << 12);
    }

    if (EXTI->PR & (1 << 15)){
        if (GPIOE->IDR & (1 << 15)){
            GPIOA->ODR |= (1 << 7);
        } else {
            GPIOA->ODR &= ~(1 << 7);
        }
        EXTI->PR |= (1 << 15);
    }
}

int main(void){
  HAL_Init();
  SystemClock_Config();
  init_clock();
  init_exti();
  init_LED();

  while (1){ }
}
