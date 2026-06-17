/**
 * @file main_I.c
 * @brief Asynchronous HMI system driven by External Interrupts (EXTI).
 * @details Implements a flag-based communication pattern between ISRs and the main 
 * loop. Decouples hardware event detection from display logic to optimize system 
 * response times.
 * @author Daniel Ruiz Perez
 * @date 2026-06-17
 */

#include "main.h"
#include "lcd.h"

// Flags para comunicacion entre ISR y bucle principal
volatile int16_t nuevaInterrupcion = 0; 
volatile uint8_t hayCambio = 0;         
volatile uint8_t flancoDetectado7 = 0;  // Estado del flanco para EXTI7
volatile uint8_t flancoDetectado15 = 0; // Estado del flanco para EXTI15

void SystemClock_Config(void);

/**
 * @brief Inicializa los relojes de los puertos B, E, A y el periférico SYSCFG.
 */
void init_clock(void){
    RCC->AHB1ENR |= ((1 << 0) | (1 << 1) | (1 << 4)); // GPIOB, E, A
    RCC->APB2ENR |= (1 << 14);                        // SYSCFG
}

/**
 * @brief Configura pines como entradas y habilita interrupciones externas (EXTI).
 */
void init_exti(void){
    // Configuración de puertos como entrada
    GPIOB->MODER &= ~((1 << 3) | (1 << 2) | (1 << 5) | (1 << 4)); 
    GPIOE->MODER &= ~0xFFFFFFFF; 

    // Pull-up/Pull-down configuración
    GPIOB->PUPDR &= ~0xFFFFFFFF;
    GPIOB->PUPDR |= ((1 << 2) | (1 << 5));
    GPIOE->PUPDR &= ~0xFFFFFFFF;
    GPIOE->PUPDR |= ((1 << 24) | (1 << 15) | (1 << 17) | (1 << 31));

    // Mapeo SYSCFG a EXTI
    SYSCFG->EXTICR[0] &= ~0xFFFF; 
    SYSCFG->EXTICR[1] &= ~0xFFFF;
    SYSCFG->EXTICR[2] &= ~0xFFFF;
    SYSCFG->EXTICR[3] &= ~0xFFFF;

    SYSCFG->EXTICR[0] |= ((1 << 4) | (1 << 8)); 
    SYSCFG->EXTICR[1] |= (1 << 14);             
    SYSCFG->EXTICR[2] |= (1 << 2);              
    SYSCFG->EXTICR[3] |= ((1 << 2) | (1 << 14));

    // Configuración de flancos
    EXTI->RTSR &= ~(0x7FFFFF);
    EXTI->FTSR &= ~(0x7FFFFF);
    EXTI->RTSR |= ((1 << 2) | (1 << 8) | (1 << 7) | (1 << 15));
    EXTI->FTSR |= ((1 << 1) | (1 << 12) | (1 << 7) | (1 << 15));

    // Habilitar interrupciones
    EXTI->IMR |= ((1 << 1) | (1 << 2) | (1 << 7) | (1 << 8) | (1 << 12) | (1 << 15));

    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_EnableIRQ(EXTI2_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void init_LED(void){
    GPIOA->MODER |= ((1 << 4) | (1 << 6) | (1 << 8) | (1 << 10) | (1 << 12) | (1 << 14));
}

// Rutinas de servicio de interrupción (ISRs)
void EXTI1_IRQHandler(void){
    if (EXTI->PR & (1 << 1)){
        for(volatile int i = 0; i < 20000; i++); // Debounce
        if (!(GPIOB->IDR & (1 << 1))) {
            GPIOA->ODR ^= (1 << 2);
            nuevaInterrupcion = 1; 
            hayCambio = 1;         
        }
        EXTI->PR |= (1 << 1);
    }
}

void EXTI2_IRQHandler(void){
    if (EXTI->PR & (1 << 2)){
        for(volatile int i = 0; i < 20000; i++);
        if (GPIOB->IDR & (1 << 2)) {
            GPIOA->ODR ^= (1 << 3);
            nuevaInterrupcion = 2;
            hayCambio = 1;
        }
        EXTI->PR |= (1 << 2);
    }
}

void EXTI9_5_IRQHandler(void){
    if (EXTI->PR & (1 << 7)){
        nuevaInterrupcion = 7;
        hayCambio = 1;
        flancoDetectado7 = (GPIOE->IDR & (1 << 7)) ? 0 : 1;
        GPIOA->ODR = (GPIOE->IDR & (1 << 7)) ? (GPIOA->ODR | (1 << 4)) : (GPIOA->ODR & ~(1 << 4));
        EXTI->PR |= (1 << 7);
    }
    if (EXTI->PR & (1 << 8)){
        for(volatile int i = 0; i < 20000; i++);
        if (GPIOE->IDR & (1 << 8)) {
            GPIOA->ODR ^= (1 << 5);
            nuevaInterrupcion = 8;
            hayCambio = 1;
        }
        EXTI->PR |= (1 << 8);
    }
}

void EXTI15_10_IRQHandler(void){
    if (EXTI->PR & (1 << 12)){
        for(volatile int i = 0; i < 20000; i++);
        if (!(GPIOE->IDR & (1 << 12))) {
            GPIOA->ODR ^= (1 << 6);
            nuevaInterrupcion = 12;
            hayCambio = 1;
        }
        EXTI->PR |= (1 << 12);
    }
    if (EXTI->PR & (1 << 15)){
        nuevaInterrupcion = 15;
        hayCambio = 1;
        flancoDetectado15 = (GPIOE->IDR & (1 << 15)) ? 0 : 1;
        GPIOA->ODR = (GPIOE->IDR & (1 << 15)) ? (GPIOA->ODR | (1 << 7)) : (GPIOA->ODR & ~(1 << 7));
        EXTI->PR |= (1 << 15);
    }
}

/**
 * @brief Maneja la lógica de actualización del LCD basada en el ID de interrupción.
 */
void message(int interrupcion){
    switch (interrupcion){
        case 1: lcd_clear(); lcd_gotoxy(1,1); lcd_puts("EXTI : 1"); lcd_gotoxy(1,2); lcd_puts("Flanco de bajada"); break;
        case 2: lcd_clear(); lcd_gotoxy(1,1); lcd_puts("EXTI : 2"); lcd_gotoxy(1,2); lcd_puts("Flanco de subida"); break;
        case 7: lcd_clear(); lcd_gotoxy(1,1); lcd_puts("EXTI : 7"); lcd_gotoxy(1,2); lcd_puts(flancoDetectado7 ? "Flanco de subida" : "Flanco de bajada"); break;
        case 8: lcd_clear(); lcd_gotoxy(1,1); lcd_puts("EXTI : 8"); lcd_gotoxy(1,2); lcd_puts("Flanco de subida"); break;
        case 12: lcd_clear(); lcd_gotoxy(1,1); lcd_puts("EXTI : 12"); lcd_gotoxy(1,2); lcd_puts("Flanco de bajada"); break;
        case 15: lcd_clear(); lcd_gotoxy(1,1); lcd_puts("EXTI : 15"); lcd_gotoxy(1,2); lcd_puts(flancoDetectado15 ? "Flanco de subida" : "Flanco de bajada"); break;
        default: lcd_clear(); lcd_gotoxy(4,1); lcd_puts("Bienvenido"); lcd_gotoxy(2,2); lcd_puts("Servicios EXTI"); break;
    }
}

int main(void){
    HAL_Init();
    SystemClock_Config();
    init_clock();
    init_LED();
    init_exti();
    lcd_init();
    message(0);

    while (1){
        if(hayCambio){
            message(nuevaInterrupcion);
            hayCambio = 0;
        }
    }
}
