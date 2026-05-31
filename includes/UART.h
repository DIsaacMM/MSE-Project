/**
 * @file UART.h
 * @brief Driver UART2 bare-metal para STM32F411RE Nucleo
 *
 * UART2 en el Nucleo está conectado al ST-Link VCP (Virtual COM Port),
 * así que puedes ver la salida directamente en cualquier terminal serial
 * (PuTTY, CoolTerm, minicom, etc.) SIN cables adicionales.
 *
 * Pines:  PA2 → TX (AF7)
 *         PA3 → RX (AF7)  ← opcional, solo si necesitas recibir
 *
 * Baudrate: 115200
 */

#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "stm32f4xx.h"

void    uart_init(void);
void    uart_sendChar(char c);
void    uart_sendString(const char *s);
void    uart_sendFloat(float val, uint8_t decimals);
void    uart_sendInt(int32_t val);
void    uart_sendLine(const char *s);   /* s + "\r\n" */

#endif
