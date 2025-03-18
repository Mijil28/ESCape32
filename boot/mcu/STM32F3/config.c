/*
** Copyright (C) Arseny Vakhrushev <arseny.vakhrushev@me.com>
**
** This firmware is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** This firmware is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this firmware. If not, see <http://www.gnu.org/licenses/>.
*/

#include "common.h"

void init(void) {
    // Aktifkan clock untuk GPIOA, GPIOB, dan CRC
    RCC_AHBENR |= RCC_AHBENR_CRCEN | RCC_AHBENR_IOPAEN | RCC_AHBENR_IOPBEN;
    RCC_APB2ENR |= RCC_APB2ENR_TIM1EN;

    // Konfigurasi Flash dan PLL
    FLASH_ACR = FLASH_ACR_LATENCY_1WS | FLASH_ACR_PRFTEN;
    RCC_CFGR = RCC_CFGR_PLLMUL_MUL12;
    RCC_CR |= RCC_CR_PLLON;
    while (!(RCC_CR & RCC_CR_PLLRDY));
    RCC_CFGR |= RCC_CFGR_SW_PLL;

    // Konfigurasi GPIO
    GPIOA_MODER &= ~0x00000000;  // Hanya bersihkan jika perlu
    GPIOB_MODER &= ~0x00000000;

#ifdef IO_PA2
    // Konfigurasi USART2 di PA2 (TX) dan PA15 (RX)
    RCC_APB1ENR |= RCC_APB1ENR_USART2EN;
    GPIOA_AFRL |= (7 << (4 * 2));  // PA2 sebagai USART2_TX (AF7)
    GPIOA_AFRH |= (7 << (4 * 7));  // PA15 sebagai USART2_RX (AF7)
    GPIOA_PUPDR |= (1 << (2 * 2)); // PA2 pull-up
    GPIOA_MODER &= ~(3 << (2 * 2)); // Clear mode bits
    GPIOA_MODER |= (2 << (2 * 2));  // PA2 sebagai alternate function

#elif defined(IO_PB9)
    // Konfigurasi USART2 di PB9 (TX)
    RCC_APB1ENR |= RCC_APB1ENR_USART2EN;
    GPIOB_AFRH |= (7 << (4 * 1));  // PB9 sebagai USART2_TX (AF7)
    GPIOB_PUPDR |= (1 << (2 * 9)); // PB9 pull-up
    GPIOB_MODER &= ~(3 << (2 * 9)); // Clear mode bits
    GPIOB_MODER |= (2 << (2 * 9));  // PB9 sebagai alternate function

#else
    // Jika tidak menggunakan USART2, gunakan PB4 untuk TIM3_CH1
    RCC_APB1ENR |= RCC_APB1ENR_TIM3EN;
    GPIOB_AFRL |= (2 << (4 * 4)); // PB4 sebagai TIM3_CH1 (AF2)
    GPIOB_PUPDR |= (1 << (2 * 4)); // PB4 pull-up
    GPIOB_MODER &= ~(3 << (2 * 4)); // Clear mode bits
    GPIOB_MODER |= (2 << (2 * 4));  // PB4 sebagai alternate function
#endif
}
