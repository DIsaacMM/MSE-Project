/**
 * @file UART.c
 * @brief Driver UART2 bare-metal — STM32F411RE Nucleo
 *
 * UART2 está cableado al ST-Link en el Nucleo → aparece como COM port en tu PC.
 * No necesitas ningún conversor externo.
 *
 * Clock: 16 MHz HSI
 * BRR para 115200: 16000000 / 115200 = 138.88 → BRR = 0x008B (mantisa=8, fracción=11)
 *   Fórmula exacta: mantisa = 16MHz / (16 * 115200) = 8
 *                   fracción = round(0.68 * 16) = 11
 *   → BRR = (8 << 4) | 11 = 0x008B
 */

#include "UART.h"
#include "GPIO.h"

/* BRR para 115200 baud a 16MHz:
 * USARTDIV = 16e6 / (16 * 115200) = 8.680...
 * mantisa  = 8       → bits [15:4]
 * fracción = 0.68*16 = 10.88 → 11  → bits [3:0]  */
#define UART2_BRR   0x008B

void uart_init(void)
{
    /* 1. Habilitar clock GPIOA y USART2 */
    RCC->AHB1ENR  |= (1U << 0);    /* GPIOA */
    RCC->APB1ENR  |= (1U << 17);   /* USART2 */

    /* 2. Configurar PA2 como AF7 (USART2_TX) */
    /* Modo Alternate Function */
    GPIOA->MODER  &= ~(3U << (2*2));
    GPIOA->MODER  |=  (2U << (2*2));
    /* AF7 en AFRL (pines 0-7) → bits [11:8] para PA2 */
    GPIOA->AFR[0] &= ~(0xFU << (4*2));
    GPIOA->AFR[0] |=  (7U   << (4*2));
    /* Pull-up, velocidad alta */
    GPIOA->PUPDR  &= ~(3U << (2*2));
    GPIOA->PUPDR  |=  (1U << (2*2));
    GPIOA->OSPEEDR|=  (3U << (2*2));

    /* 3. Configurar USART2 */
    USART2->CR1 = 0;                /* Reset */
    USART2->BRR = UART2_BRR;        /* 115200 baud @ 16MHz */
    USART2->CR1 = (1U << 3)         /* TE: transmit enable */
                | (1U << 13);       /* UE: USART enable */
}

void uart_sendChar(char c)
{
    /* Esperar a que el registro de transmisión esté vacío (TXE=1) */
    while (!(USART2->SR & (1U << 7)));
    USART2->DR = (uint8_t)c;
}

void uart_sendString(const char *s)
{
    while (*s) uart_sendChar(*s++);
}

void uart_sendLine(const char *s)
{
    uart_sendString(s);
    uart_sendChar('\r');
    uart_sendChar('\n');
}

void uart_sendInt(int32_t val)
{
    char buf[12];
    int8_t i = 0;
    if (val < 0) { uart_sendChar('-'); val = -val; }
    if (val == 0) { uart_sendChar('0'); return; }
    while (val > 0) { buf[i++] = '0' + (val % 10); val /= 10; }
    while (i > 0) uart_sendChar(buf[--i]);
}

void uart_sendFloat(float val, uint8_t decimals)
{
    if (val < 0) { uart_sendChar('-'); val = -val; }
    /* Parte entera */
    uart_sendInt((int32_t)val);
    uart_sendChar('.');
    /* Parte decimal */
    val -= (int32_t)val;
    for (uint8_t d = 0; d < decimals; d++) {
        val *= 10.0f;
        uart_sendChar('0' + (uint8_t)val);
        val -= (uint8_t)val;
    }
}
