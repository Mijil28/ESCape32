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

/*void init(void) {
    // Aktifkan clock untuk GPIOA, GPIOB, dan CRC
    RCC_AHBENR |= RCC_AHBENR_CRCEN | RCC_AHBENR_IOPAEN | RCC_AHBENR_IOPBEN;
    RCC_APB2ENR |= RCC_APB2ENR_TIM1EN;

    // Konfigurasi Flash dan PLL
    FLASH_ACR = FLASH_ACR_LATENCY_MASK | FLASH_ACR_PRFTEN;
    RCC_CFGR = RCC_CFGR_PLLMUL_MUL12;
    RCC_CR |= RCC_CR_PLLON;
    while (!(RCC_CR & RCC_CR_PLLRDY));
    RCC_CFGR |= RCC_CFGR_SW_PLL;

    // Konfigurasi GPIO
    GPIOA_MODER &= ~0x00000000;  // Hanya bersihkan jika perlu
    GPIOB_MODER &= ~0x00000000;

    // Konfigurasi USART2 di PB9 (TX)
    RCC_APB1ENR |= RCC_APB1ENR_USART2EN;
    GPIOB_AFRH |= (7 << (4 * 1));  // PB9 sebagai USART2_TX (AF7)
    GPIOB_PUPDR |= (1 << (2 * 9)); // PB9 pull-up
    GPIOB_MODER &= ~(3 << (2 * 9)); // Clear mode bits
    GPIOB_MODER |= (2 << (2 * 9));  // PB9 sebagai alternate function

}*/

void init(void) {
    // Aktifkan clock untuk GPIO dan USART2
    RCC_AHBENR |= RCC_AHBENR_CRCEN | RCC_AHBENR_IOPAEN | RCC_AHBENR_IOPBEN;
    RCC_APB1ENR = RCC_APB1ENR_USART2EN;
    RCC_APB2ENR = RCC_APB2ENR_TIM1EN;

    // Konfigurasi Flash dan PLL untuk STM32F3
    FLASH_ACR = FLASH_ACR_LATENCY_MASK | FLASH_ACR_PRFTBE;
    RCC_CFGR = RCC_CFGR_PLLMUL_MUL12 | RCC_CFGR_PLLSRC_HSI_DIV2;
    RCC_CR |= RCC_CR_PLLON;
    while (!(RCC_CR & RCC_CR_PLLRDY));
    RCC_CFGR |= RCC_CFGR_SW_PLL;

    // Konfigurasi PB9 sebagai USART2_TX (AF7)
    GPIOB_AFRH &= ~(0xF << (4 * 1));  // Hapus AF sebelumnya
    GPIOB_AFRH |= (7 << (4 * 1));     // PB9 sebagai AF7 (USART2_TX)

    // Konfigurasi PA15 sebagai USART2_RX (AF3)
    GPIOA_AFRH &= ~(0xF << (4 * 7));  
    GPIOA_AFRH |= (3 << (4 * 7));     // PA15 sebagai AF3 (USART2_RX)

    // Konfigurasi Pull-up untuk komunikasi serial
    GPIOB_PUPDR &= ~(3 << (2 * 9));   // Hapus pull-up PB9
    GPIOB_PUPDR |= (1 << (2 * 9));    // Set PB9 pull-up

    GPIOA_PUPDR &= ~(3 << (2 * 15));  // Hapus pull-up PA15
    GPIOA_PUPDR |= (1 << (2 * 15));   // Set PA15 pull-up

    // Set mode ke Alternate Function
    GPIOB_MODER &= ~(3 << (2 * 9));   // Hapus mode PB9
    GPIOB_MODER |= (2 << (2 * 9));    // PB9 sebagai alternate function

    GPIOA_MODER &= ~(3 << (2 * 15));  // Hapus mode PA15
    GPIOA_MODER |= (2 << (2 * 15));   // PA15 sebagai alternate function
}

