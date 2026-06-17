/**
 * @file main.c
 * @brief Hardware PWM Generation via Timer 4 on STM32F407VGT6.
 * @details Bypasses HAL abstractions to directly configure the Advanced-Control 
 * Timers at the register level. Maps TIM4 channels 1-4 to GPIOD pins 12-15 
 * using Alternate Function (AF2) to drive hardware PWM outputs independently.
 * @author Daniel Ruiz Perez
 * @date 2026-04-27
 */

#include "main.h"

void SystemClock_Config(void);

/**
 * @brief Initializes Timer 4 and GPIOD to generate hardware PWM on 4 channels.
 */
void init_timer4_pwm(void){
    // 1. Enable Clocks for GPIOD (AHB1) and TIM4 (APB1)
    RCC->AHB1ENR |= (1 << 3);                                               // GPIOD Clock Enable
    RCC->APB1ENR |= (1 << 2);                                               // TIM4 Clock Enable

    // 2. Configure GPIOD Pins 12, 13, 14, 15 as Alternate Function
    GPIOD->MODER &= ~((3U << 24) | (3U << 26) | (3U << 28) | (3U << 30));   // Clear Mode bits
    GPIOD->MODER |=  ((2U << 24) | (2U << 26) | (2U << 28) | (2U << 30));   // Set to AF mode (10)

    // 3. Map Alternate Function 2 (AF2) for TIM4 on Pins 12-15
    // AFR[1] controls pins 8 to 15. AF2 is binary '0010' (2U).
    GPIOD->AFR[1] &= ~((15U << 16) | (15U << 20) | (15U << 24) | (15U << 28));
    GPIOD->AFR[1] |=  ((2U << 16)  | (2U << 20)  | (2U << 24)  | (2U << 28));

    // 4. Configure Timer 4 Timebase (Frequency)
    // Assuming 16MHz clock. Prescaler = 16. Clock = 1MHz. ARR = 1000. PWM Freq = 1kHz.
    TIM4->PSC = 16 - 1;
    TIM4->ARR = 1000 - 1;
    TIM4->CR1 &= ~(1 << 4);                                                 // Up-counting mode

    // 5. Configure Output Compare Mode (PWM Mode 1)
    TIM4->CCMR1 &= ~(0xFFFF);                                               // Clear CCMR1
    TIM4->CCMR1 &= ~((3 << 0) | (3 << 8));                                  // CC1/CC2 as outputs
    TIM4->CCMR1 |=  ((6 << 4) | (6 << 12));                                 // PWM Mode 1 for CH1 & CH2 (110)

    TIM4->CCMR2 &= ~(0xFFFF);                                               // Clear CCMR2
    TIM4->CCMR2 &= ~((3 << 0) | (3 << 8));                                  // CC3/CC4 as outputs
    TIM4->CCMR2 |=  ((6 << 4) | (6 << 12));                                 // PWM Mode 1 for CH3 & CH4 (110)

    // 6. Enable Output Channels
    // Bit 0: CC1E, Bit 4: CC2E, Bit 8: CC3E, Bit 12: CC4E
    TIM4->CCER |= ((1 << 0) | (1 << 4) | (1 << 8) | (1 << 12));

    // 7. Start the Timer Counter
    TIM4->CNT = 0;
    TIM4->CR1 |= (1 << 0);                                                  // Enable Counter (CEN)
}

int main(void){
    HAL_Init();
    SystemClock_Config();
    init_timer4_pwm();

    while (1){
        // Smoothly fade LEDs in and out utilizing the hardware PWM
        for (int i = 0; i <= 1000; i++){
            TIM4->CCR1 = i;
            TIM4->CCR2 = i;
            TIM4->CCR3 = i;
            TIM4->CCR4 = i;
            HAL_Delay(1);
        }
        for (int i = 1000; i >= 0; i--){
            TIM4->CCR1 = i;
            TIM4->CCR2 = i;
            TIM4->CCR3 = i;
            TIM4->CCR4 = i;
            HAL_Delay(1);
        }
    }
}
