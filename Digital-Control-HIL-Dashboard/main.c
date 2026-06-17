/**
 * @file main.c
 * @brief Deterministic Motor Control & Telemetry Firmware for STM32F407.
 * @details Implements a hard-real-time 20ms control loop using TIM6. Features hardware 
 * quadrature encoder decoding (TIM3), hardware PWM generation (TIM4), and an asynchronous 
 * non-blocking UART ring buffer for bi-directional HIL (Hardware-in-the-Loop) telemetry.
 * Supports PID and Discrete Lead-Lag Compensators via difference equations.
 * @author Daniel Ruiz Perez
 * @date 2026-06-15
 */

#include "main.h"
#include "lcd.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#define MAX_BUFFER 64

volatile char rx_buffer[MAX_BUFFER];
volatile uint8_t rx_index = 0;
volatile uint8_t string_ready = 0;

// --- VARIABLES PARA TX POR INTERRUPCIÓN ---
#define TX_BUFFER_SIZE 256
volatile char tx_buffer[TX_BUFFER_SIZE];
volatile uint16_t tx_head = 0;
volatile uint16_t tx_tail = 0;

// --- VARIABLES DE ESTADO DEL CONTROLADOR ---
volatile uint8_t sistema_corriendo = 0;
char modo_actual[4];
volatile uint8_t id_controlador = 0xFF;
volatile float setpoint_actual = 0;
volatile float K1_MCU = 0.0;
volatile float K2_MCU = 0.0;
volatile float K3_MCU = 0.0;
volatile float K4_MCU = 0.0; // <-- NUEVA GANANCIA 4
volatile float K5_MCU = 0.0; // <-- NUEVA GANANCIA 5

// --- Variables Globales de Control ---
float resolution4x = 1364.8f;
long posicionPulsos = 0;
uint16_t last_count = 0;

// Filtro RPM
#define VENTANA 10
float lecturasRPM[VENTANA] = {0};
int indiceFiltro = 0;
float sumaRPM = 0;

// Variables de actuación del motor
int pwm_duty = 0;
int pwm_step = 20;
const int PWM_MAX = 1000;

// --- Variables de Comunicación ---
volatile bool nuevos_datos = false;
volatile float angulo_compartido = 0.0f;
volatile float rpm_compartido = 0.0f;

// --- Memorias del Controlador Digital ---
volatile float e_k1 = 0.0f;
volatile float e_k2 = 0.0f;
volatile float u_k1 = 0.0f;
volatile float u_k2 = 0.0f;
volatile float integral_acumulada = 0.0f;

// --- Memorias del Autotuning (Relevador) ---
volatile float auto_max_A = 0.0f;          // Amplitud pico (A)
volatile uint32_t auto_timer_ticks = 0;    // Contador de tiempo para el Periodo
volatile uint8_t auto_zero_crossings = 0;  // Contador de cruces por cero
volatile bool auto_last_sign = false;      // Signo anterior del error
volatile float auto_Ku = 0.0f;             // Ganancia Crítica calculada
volatile float auto_Pu = 0.0f;             // Periodo Crítico calculado
volatile bool autotune_completado = false; // Bandera para dejar de medir
volatile bool enviar_resultados_auto = false; // Bandera para enviar por UART

void SystemClock_Config(void);

// CONFIGURACIÓN DE HARDWARE 
void Init_Encoder_TIM3(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    GPIOA->MODER &= ~((3U << 12) | (3U << 14));
    GPIOA->MODER |=  ((2U << 12) | (2U << 14));
    GPIOA->AFR[0] &= ~((15U << 24) | (15U << 28));
    GPIOA->AFR[0] |=  ((2U << 24) | (2U << 28));
    TIM3->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
    TIM3->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
    TIM3->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
    TIM3->ARR = 65535;
    TIM3->CR1 |= TIM_CR1_CEN;
}

void Init_Motor_Control(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    GPIOD->MODER &= ~(3U << 24);
    GPIOD->MODER |=  (2U << 24);
    GPIOD->AFR[1] &= ~(15U << 16);
    GPIOD->AFR[1] |=  (2U << 16);

    TIM4->PSC = 0;
    TIM4->ARR = 1000 - 1; // Resolución de 0 a 1000 (16 kHz)
    TIM4->CCMR1 |= (6U << 4);
    TIM4->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM4->CCER |= TIM_CCER_CC1E;
    TIM4->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;

    // PD13 y PD14 (Dirección), PD15 (STBY) como salidas discretas
    GPIOD->MODER &= ~((3U << 26) | (3U << 28) | (3U << 30));
    GPIOD->MODER |=  ((1U << 26) | (1U << 28) | (1U << 30));
}

void Init_Control_Loop_Timer(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC = 16000 - 1;
    TIM6->ARR = 20 - 1; // Interrupción periódica estricta cada 20ms
    TIM6->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    TIM6->CR1 |= TIM_CR1_CEN;
}

void init_uart2(void){
	RCC->AHB1ENR |= (1 << 0);
	RCC->APB1ENR |= (1 << 17);

	GPIOA->MODER &= ~((3 << 4) | (3 << 6));
	GPIOA->MODER |=  ((2 << 4) | (2 << 6));
	GPIOA->PUPDR &= ~((3 << 4) | (3 << 6));
	GPIOA->OTYPER &= ~(1 << 2);

	GPIOA->AFR[0] &= ~((15 << 8) | (15 << 12));
	GPIOA->AFR[0] |=   ((7 << 8) | (7 << 12));

	USART2->BRR = (8 << 4) | 11; // 115200 baudios con APB1 a 16MHz

	USART2->CR1 = 0;
	USART2->CR1 &= ~(1 << 15);
	USART2->CR1 &= ~(1 << 12);
	USART2->CR1 &= ~(1 << 10);
	USART2->CR2 &= ~(3 << 12);

	USART2->CR1 |= (1 << 3) | (1 << 2) | (1 << 5) | (1 << 13);

	NVIC_SetPriority(USART2_IRQn, 0);
    NVIC_EnableIRQ(USART2_IRQn);
}

void usart_send_async(const char *str) {
    while (*str) {
        uint16_t next_head = (tx_head + 1) % TX_BUFFER_SIZE;
        while (next_head == tx_tail) {
             USART2->CR1 |= (1 << 7); // Forzar vaciado si el búfer se llena
        }
        tx_buffer[tx_head] = *str;
        tx_head = next_head;
        str++;
    }
    USART2->CR1 |= (1 << 7); // Habilitar la interrupción TXE para transmisión asíncrona
}

void Init_Hardware(void) {
    Init_Encoder_TIM3();
    Init_Motor_Control();
    init_uart2();
}

// INTERRUPCIONES DETALLES
void USART2_IRQHandler(void) {
    // 1. RECEPCIÓN (RX)
    if (USART2->SR & (1 << 5)) {
        char k = (char)USART2->DR;
        if (string_ready == 0) {
            if (k == '<') {
                rx_index = 0;
            }
            else if (k == '>') {
                rx_buffer[rx_index] = '\0';
                string_ready = 1;
            }
            else if (rx_index < (MAX_BUFFER - 1)) {
                rx_buffer[rx_index++] = k;
            }
        }
    }

    // 2. TRANSMISIÓN (TX)
    if ((USART2->CR1 & (1 << 7)) && (USART2->SR & (1 << 7))) {
        if (tx_head != tx_tail) {
            USART2->DR = tx_buffer[tx_tail];
            tx_tail = (tx_tail + 1) % TX_BUFFER_SIZE;
        } else {
            USART2->CR1 &= ~(1 << 7); // Desactivar interrupción si el búfer está vacío
        }
    }
}

void TIM6_DAC_IRQHandler(void) {
    if (TIM6->SR & TIM_SR_UIF) {
        TIM6->SR &= ~TIM_SR_UIF; // Bajar la bandera de hardware de inmediato

        // 1. LECTURA DE SENSORES (Multi-vuelta y Bidireccional)
        uint16_t current_count = TIM3->CNT;
        int16_t pulsosInstantaneos = (int16_t)(current_count - last_count);
        last_count = current_count;

        posicionPulsos += pulsosInstantaneos;

        // --- Cálculo de RPM (SIN valor absoluto para soportar reversa real) ---
        float rpmInstantanea = ((float)pulsosInstantaneos / resolution4x) * (60.0f / 0.02f);

        // Filtro de media móvil para suavizar la velocidad
        sumaRPM -= lecturasRPM[indiceFiltro];
        lecturasRPM[indiceFiltro] = rpmInstantanea;
        sumaRPM += lecturasRPM[indiceFiltro];
        indiceFiltro = (indiceFiltro + 1) % VENTANA;
        rpm_compartido = sumaRPM / (float)VENTANA;

        // --- Cálculo de Posición Angular ABSOLUTA (Sin límite de 360°) ---
        // Esto permite rampas infinitas y números negativos continuos
        float anguloTotal = (float)posicionPulsos * 360.0f / resolution4x;
        angulo_compartido = anguloTotal;

        // 2. ESTRATEGIA DE CONTROL MATEMÁTICO
        if (sistema_corriendo == 1) {

            // Decidir qué variable estamos controlando según la GUI
            float lectura_actual = (strncmp((char*)modo_actual, "POS", 3) == 0) ? angulo_compartido : rpm_compartido;

            // Cálculo del Error Continuo
            float error = setpoint_actual - lectura_actual;
            float esfuerzo = 0.0f;
            float dt = 0.02f; // El tiempo de muestreo fijo de 20 milisegundos

            // --- SELECCIÓN DEL ALGORITMO DE CONTROL ---
            switch (id_controlador) {
                case 0: // CONTROL P
                    esfuerzo = K1_MCU * error;
                    break;

                case 1: // CONTROL PI
                    integral_acumulada += error * dt;
                    esfuerzo = (K1_MCU * error) + (K2_MCU * integral_acumulada);
                    break;

                case 2: // CONTROL PD
                    esfuerzo = (K1_MCU * error) + (K3_MCU * (error - e_k1) / dt);
                    break;

                case 3: // CONTROL PID
                    integral_acumulada += error * dt;
                    esfuerzo = (K1_MCU * error) + (K2_MCU * integral_acumulada) + (K3_MCU * (error - e_k1) / dt);
                    break;

                case 4: // COMPENSADORES DISCRETOS (1er Orden)
                	esfuerzo = (K1_MCU * error) + (K2_MCU * e_k1) - (K3_MCU * u_k1);
                	break;
                case 5:
                	esfuerzo = (K1_MCU * error) + (K2_MCU * e_k1) - (K3_MCU * u_k1);
                	break;
                case 6:
                	esfuerzo = (K1_MCU * error) + (K2_MCU * e_k1) + (K3_MCU * e_k2) - (K4_MCU * u_k1) - (K5_MCU * u_k2);
                	break;
                case 7: // AUTOTUNING (Método de Relevador)

                	// --- 1. DEFINIR LA AMPLITUD DEL RELEVADOR (d) ---
                	// Como la GUI solo manda el Modo 7, definimos 'd' aquí.
                	// 500 de 1000 es el 50% de la potencia máxima.
                	float AMPLITUD_RELE = 500.0f;

                	// Lógica On-Off (Relevador)
                	if (error > 0.0f) {
                		esfuerzo = AMPLITUD_RELE;  // Empuje hacia adelante
                	} else if (error < 0.0f) {
                		esfuerzo = -AMPLITUD_RELE; // Empuje en reversa
                	} else {
                		esfuerzo = 0.0f;
                	}

                	// --- 2. MÁQUINA DE ESTADOS PARA MEDIR Ku Y Pu ---
                	if (!autotune_completado) {
                		bool current_sign = (error >= 0.0f);

                		// Configuración inicial en el primer instante
                		if (auto_zero_crossings == 0 && error != 0.0f) {
                			auto_last_sign = current_sign;
                			auto_zero_crossings = 1;
                		}

                		// Rastrear el pico máximo absoluto (Amplitud A)
                		if (fabs(error) > auto_max_A) {
                			auto_max_A = fabs(error);
                		}

                		// Contar el tiempo (cada tick son 20ms)
                		if (auto_zero_crossings > 1) {
                			auto_timer_ticks++;
                		}

                		// Detectar un Cruce por Cero
                		if (auto_zero_crossings > 0 && current_sign != auto_last_sign) {
                			auto_zero_crossings++;
                			auto_last_sign = current_sign;

                			if (auto_zero_crossings == 2) {
                				// Descartamos el primer medio ciclo (suele ser asimétrico)
                				auto_timer_ticks = 0;
                				auto_max_A = 0.0f;
                			}
                			else if (auto_zero_crossings == 6) {
                				// Han pasado 2 ciclos completos
                				auto_Pu = (auto_timer_ticks * 0.02f) / 2.0f;

                				// FÓRMULA DE FOURIER USANDO NUESTRA CONSTANTE
                				// Ku = 4d / (pi * A)
                				auto_Ku = (4.0f * AMPLITUD_RELE) / (3.14159265f * auto_max_A);

                				autotune_completado = true;
                				enviar_resultados_auto = true;
                			}
                		}
                	}
                	break;
            }

            // --- SATURACIÓN Y PROTECCIÓN ANTI-WINDUP ---
            if (esfuerzo > PWM_MAX) { // Límite superior (+1000)
                esfuerzo = PWM_MAX;
                // Anti-Windup: Evitar que la integral crezca al infinito si el motor no puede dar más de sí
                if (id_controlador == 1 || id_controlador == 3) integral_acumulada -= error * dt;

            } else if (esfuerzo < -PWM_MAX) { // Límite inferior (-1000)
                esfuerzo = -PWM_MAX;
                // Anti-Windup en reversa
                if (id_controlador == 1 || id_controlador == 3) integral_acumulada -= error * dt;
            }

            // --- ACTUACIÓN EN EL HARDWARE (Lógica de 3 Estados) ---
            if (esfuerzo == 0.0f) {
                // PUNTO MUERTO (Coast): Desconecta el motor si no se requiere esfuerzo
                GPIOD->ODR &= ~((1 << 13) | (1 << 14));
                pwm_duty = 0;
            }
            else if (esfuerzo > 0.0f) {
                // GIRO ADELANTE
                GPIOD->ODR |= (1 << 13);
                GPIOD->ODR &= ~(1 << 14);
                pwm_duty = (int)esfuerzo;
            }
            else {
                // GIRO EN REVERSA
                GPIOD->ODR &= ~(1 << 13);
                GPIOD->ODR |= (1 << 14);
                pwm_duty = (int)(-esfuerzo);  // El PWM requiere un número positivo absoluto
            }

            // Escribir la potencia final en el registro del Timer 4
            TIM4->CCR1 = pwm_duty;

            // --- ACTUALIZACIÓN DE MEMORIAS (Para el ciclo discreto k+1) ---
            e_k2 = e_k1;
            e_k1 = error;
            u_k2 = u_k1;
            u_k1 = esfuerzo;
        }

        // Levantar la bandera asíncrona para que el bucle Main() dispare la telemetría a Python
        nuevos_datos = true;
    }
}

// FUNCIONES DE CONTROL PRINCIPALES
void procesar_comandos_gui(void) {
    if (string_ready == 1) {
        if (strncmp((char*)rx_buffer, "STOP", 4) == 0) {
            sistema_corriendo = 0;
            id_controlador = 0xFF;

            GPIOD->ODR &= ~((1 << 13) | (1 << 14) | (1 << 15));
            TIM4->CCR1 = 0;

            posicionPulsos = 0;
            TIM3->CNT = 0;
            last_count = 0;

            e_k1 = 0.0f; e_k2 = 0.0f;
            u_k1 = 0.0f; u_k2 = 0.0f;
            integral_acumulada = 0.0f;

            indiceFiltro = 0;
            sumaRPM = 0.0f;
            for (int i = 0; i < VENTANA; i++) lecturasRPM[i] = 0.0f;
            rpm_compartido = 0.0f;
            angulo_compartido = 0.0f;

            // Reinicio de Autotuning
            auto_max_A = 0.0f; auto_timer_ticks = 0; auto_zero_crossings = 0;
            autotune_completado = false; enviar_resultados_auto = false;
        }
        else if (strncmp((char*)rx_buffer, "START", 5) == 0) {
            char *token = strtok((char*)rx_buffer, ",");
            if (token != NULL) {
                token = strtok(NULL, ","); if (token) strcpy(modo_actual, token);
                token = strtok(NULL, ","); if (token) id_controlador = atoi(token);
                token = strtok(NULL, ","); if (token) K1_MCU = atof(token);
                token = strtok(NULL, ","); if (token) K2_MCU = atof(token);
                token = strtok(NULL, ","); if (token) K3_MCU = atof(token);
                token = strtok(NULL, ","); if (token) K4_MCU = atof(token);
                token = strtok(NULL, ","); if (token) K5_MCU = atof(token);

                if (sistema_corriendo == 0) {
                	pwm_duty = 0;
                	TIM4->CCR1 = 0;
                	GPIOD->ODR &= ~((1 << 13) | (1 << 14));
                	GPIOD->ODR |= (1 << 15);
                	// Reinicio de Autotuning al arrancar
                	auto_max_A = 0.0f; auto_timer_ticks = 0; auto_zero_crossings = 0;
                	autotune_completado = false; enviar_resultados_auto = false;
                	sistema_corriendo = 1;
                }
            }
        }
        else if (strncmp((char*)rx_buffer, "SP", 2) == 0) {
            char *token = strtok((char*)rx_buffer, ",");
            token = strtok(NULL, ",");
            if (token) setpoint_actual = atof(token);
        }
        string_ready = 0;
    }
}

int main(void){
    HAL_Init();
    SystemClock_Config();
    Init_Hardware();

    // Iniciar con hardware complemente aislado
    GPIOD->ODR &= ~((1 << 13) | (1 << 14) | (1 << 15));

    Init_Control_Loop_Timer();

    while (1) {
    	procesar_comandos_gui();

    	if (nuevos_datos == true) {
    		nuevos_datos = false;

    		if (sistema_corriendo == 1) {
    			// 1. SIEMPRE ENVIAR LA TELEMETRÍA (Para graficar la oscilación)
    			char telemetria[64];
    			float lectura_actual = (strncmp((char*)modo_actual, "POS", 3) == 0) ? angulo_compartido : rpm_compartido;
    			float error_actual = setpoint_actual - lectura_actual;

    			snprintf(telemetria, sizeof(telemetria), "T,%.2f,%.2f,%d\n", lectura_actual, error_actual, pwm_duty);

    			usart_send_async(telemetria);

    			// 2. ENVIAR RESULTADOS DE AUTOTUNING (Solo se dispara una vez por prueba)
    			if (enviar_resultados_auto) {
    				enviar_resultados_auto = false; // Bajar bandera
    				char auto_msg[64];
    				// Enviamos la trama especial: <AUTO,Ku,Pu>
    				snprintf(auto_msg, sizeof(auto_msg), "<AUTO,%.4f,%.4f>\n", auto_Ku, auto_Pu);
    				usart_send_async(auto_msg);
    			}
    		}
    	}
	}
}
