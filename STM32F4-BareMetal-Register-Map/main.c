/**
 * @file main.c
 * @brief Bare-metal register mapping and GPIO control for STM32F407VGT6.
 * @details Bypasses vendor HAL and CMSIS headers by manually defining memory boundaries, 
 * AHB1 bus offsets, and peripheral structures. Implements a hardware-debounced 
 * LED sequence utilizing precise bitwise register operations.
 * @author Daniel Ruiz Perez
 * @date 2024-06-25
 */

#include <stdint.h>

// ---------------- Memory Map & Bus Offsets ----------------
#define PERIPHERALS         0x40000000UL
#define AHB1_OFFSET         0x00020000UL
#define AHB1                (PERIPHERALS + AHB1_OFFSET)

// ---------------- Peripheral Offsets ----------------
#define GPIOA_OFFSET        0x00000000UL
#define GPIOB_OFFSET        0x00000400UL
#define GPIOC_OFFSET        0x00000800UL
#define GPIOD_OFFSET        0x00000C00UL
#define RCC_OFFSET          0x00003800UL

// ---------------- Peripheral Base Addresses ----------------
#define GPIOA_ADDRESS       (AHB1 + GPIOA_OFFSET)
#define GPIOB_ADDRESS       (AHB1 + GPIOB_OFFSET)
#define GPIOC_ADDRESS       (AHB1 + GPIOC_OFFSET)
#define GPIOD_ADDRESS       (AHB1 + GPIOD_OFFSET)
#define RCC_ADDRESS         (AHB1 + RCC_OFFSET)

// ---------------- Hardware Register Structures ----------------

/**
 * @brief GPIO Register Map mapped precisely to the STM32F4 Reference Manual.
 */
typedef struct {
    volatile uint32_t MODER;    // 0x00
    volatile uint32_t OTYPER;   // 0x04
    volatile uint32_t OSPEEDR;  // 0x08
    volatile uint32_t PUPDR;    // 0x0C
    volatile uint32_t IDR;      // 0x10
    volatile uint32_t ODR;      // 0x14
    volatile uint32_t BSRR;     // 0x18 (Corrected from BCR for Cortex-M4 architecture)
    volatile uint32_t LCKR;     // 0x1C
    volatile uint32_t AFRL;     // 0x20
    volatile uint32_t AFRH;     // 0x24
} GPIO_typedef;

/**
 * @brief Reset and Clock Control (RCC) Register Map.
 * @details Utilizes dummy arrays to pad memory to the exact 0x30 offset required for AHB1ENR.
 */
typedef struct {
    volatile uint32_t DUMMY1[12]; // Pads 0x00 to 0x2C
    volatile uint32_t AHB1ENR;    // 0x30
    volatile uint32_t DUMMY2[21]; 
} RCC_typedef;

// ---------------- Peripheral Pointers ----------------
#define RCC     ((RCC_typedef*)(RCC_ADDRESS))
#define GPIOA   ((GPIO_typedef*)(GPIOA_ADDRESS))
#define GPIOB   ((GPIO_typedef*)(GPIOB_ADDRESS))
#define GPIOC   ((GPIO_typedef*)(GPIOC_ADDRESS))
#define GPIOD   ((GPIO_typedef*)(GPIOD_ADDRESS))

int main(void) {
    // 1. Enable Clock for GPIOA and GPIOD on the AHB1 bus
    RCC->AHB1ENR |= ((1 << 0) | (1 << 3));

    // 2. Configure GPIOA (Pins 0 to 5 strictly as outputs, rest unused)
    GPIOA->MODER &= ~(0x00000FFFUL);          // Clear Mode bits for PA0-PA5
    GPIOA->MODER |= 0x00000555UL;           // Set Mode bits to 01 (General Purpose Output)
    GPIOA->OTYPER &= ~(0x0000003FUL);         // Ensure Push-Pull configuration (0)

    // 3. Configure GPIOD (Pin 0 as Input for Button)
    GPIOD->MODER &= ~(0x00000003UL);          // Clear Mode bits for PD0 (Input Mode)

    int paso = 0;
    int boton_presionado = 0;

    while(1) {
        // Read Input Data Register on PD0
        if (GPIOD->IDR & (1 << 0)) {
            // Volatile keyword prevents GCC optimizer from deleting the delay loop
            for(volatile int i = 0; i < 50000; i++); 
            
            if(!boton_presionado) {
                if (paso <= 5) {
                    GPIOA->ODR |= (1 << paso); // Turn on LED sequentially
                    paso++;
                } else {
                    GPIOA->ODR &= ~(0x0000003FUL); // Clear PA0-PA5 entirely
                    paso = 0;
                }
                boton_presionado = 1;
            }
        } else {
            boton_presionado = 0;
        }
    }
}
