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

#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f1/adc.h>
#include "common.h"

// Sensor channel mapping
#if SENS_MAP == 0xA3
    #define SENS_CHAN 0x3
#elif SENS_MAP == 0xA6A0
    #define SENS_CHAN 0xc0
#endif

// Analog and temperature channel definitions
#ifndef ANALOG_CHAN
    #define ANALOG_CHAN 0x2 // ADC_IN2 (PA2)
#endif

#ifndef TEMP_CHAN
    #define TEMP_CHAN 0x10 // ADC_IN16 (temp)
    #define TEMP_FUNC(x) (((1440 - (x)) * 2000 >> 11) + 100)
#endif

#define ADC1_BASE ADC_BASE
#define COMP_CSR MMIO32(SYSCFG_COMP_BASE + 0x1c)

#ifdef USE_COMP2
    #define COMP_SHIFT 16
#else
    #define COMP_SHIFT 0
#endif

// Global variables
static char len, ain;
static uint16_t buf[6];

// Function prototypes
void init(void);
void compctl(int x);
void io_serial(void);
void io_analog(void);
void adctrig(void);
void tim1_brk_up_trg_com_isr(void);
void tim1_cc_isr(void);
void dma1_channel1_isr(void);

// Initialization function
void init(void) {
    // Reset and enable clocks
    RCC_APB2RSTR = -1;
    RCC_APB1RSTR = -1;
    RCC_APB2RSTR = 0;
    RCC_APB1RSTR = 0;
    RCC_AHBENR = RCC_AHBENR_DMAEN | RCC_AHBENR_SRAMEN | RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
    RCC_APB2ENR = RCC_APB2ENR_SYSCFGCOMPEN | RCC_APB2ENR_ADCEN | RCC_APB2ENR_TIM1EN | RCC_APB2ENR_USART1EN;
    RCC_APB1ENR = RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM6EN | RCC_APB1ENR_WWDGEN;

    // Set vector table address
    SCB_VTOR = (uint32_t)_rom;

    // Configure system clock
    RCC_CFGR &= ~RCC_CFGR_SW_PLL;
    while (RCC_CFGR & RCC_CFGR_SWS_PLL);
    RCC_CR &= ~RCC_CR_PLLON;
    while (RCC_CR & RCC_CR_PLLRDY);
    RCC_CFGR = 0x8240000; // PLLMUL=11001 (x26)
    RCC_CR |= RCC_CR_PLLON;
    while (!(RCC_CR & RCC_CR_PLLRDY));
    RCC_CFGR |= RCC_CFGR_SW_PLL;

    // Default GPIO state - analog input
    GPIOA_AFRH = 0x00000222;
    GPIOB_AFRL = 0x00006600;
    GPIOB_PUPDR = 0x00001000;
    GPIOA_MODER = 0xebeabfff;
    GPIOB_MODER = 0xffffeffa;

    // Enable USART and Timer interrupts
    nvic_set_priority(NVIC_USART1_IRQ, 0x80);
    nvic_set_priority(NVIC_USART2_IRQ, 0x40);
    nvic_enable_irq(NVIC_USART1_IRQ);
    nvic_enable_irq(NVIC_USART2_IRQ);

    // ADC initialization
    RCC_CR2 |= RCC_CR2_HSI14ON;
    while (!(RCC_CR2 & RCC_CR2_HSI14RDY));
    ADC1_CR2 = ADC_CR2_ADON | ADC_CR2_TSVREFE;
    ADC1_CR2 |= ADC_CR2_CAL;
    while (ADC1_CR2 & ADC_CR2_CAL);
    ADC1_CR1 = ADC_CR1_SCAN;
    ADC1_SMPR1 = -1;
    ADC1_SMPR2 = -1;
    ADC1_SQR3 = SENS_CHAN;
    
    len = SENS_CNT;
    if (IO_ANALOG) {
        ADC1_SQR3 |= ANALOG_CHAN << (len++ * 5);
        ain = 1;
    }
    ADC1_SQR3 |= (TEMP_CHAN | 0x220) << (len * 5);
    len += 2;
    ADC1_SQR1 = (len - 1) << ADC_SQR1_L_LSB;

    DMA1_CPAR(1) = (uint32_t)&ADC1_DR;
    DMA1_CMAR(1) = (uint32_t)buf;
}

// Comparator control
void compctl(int x) {
    int cr = 0;
    switch (x & 3) {
        case COMP_IN1:
            cr = 0x61;
            break;
        case COMP_IN2:
            cr = 0x41;
            break;
        case COMP_IN3:
            cr = 0x51;
            break;
    }
    if (x & 4) cr |= 0x800;
    COMP_CSR = cr << COMP_SHIFT;
}

// Serial interface initialization
void io_serial(void) {
    RCC_APB1ENR |= RCC_APB1ENR_USART2EN;
    GPIOA_AFRL |= 0x100;
    GPIOB_AFRL |= 0x00000040;
}

// Analog input configuration
void io_analog(void) {
    GPIOA_PUPDR &= ~0x30;
    GPIOA_MODER |= 0x30;
}

// ADC trigger function
void adctrig(void) {
    if (DMA1_CCR(1) & DMA_CCR_EN) return;
    DMA1_CNDTR(1) = len;
    DMA1_CCR(1) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_MINC | DMA_CCR_PSIZE_16BIT | DMA_CCR_MSIZE_16BIT;
    ADC1_CR2 |= ADC_CR2_DMA | ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_SWSTART;
}

// TIM1 update ISR
void tim1_brk_up_trg_com_isr(void) {
    if (TIM1_SR & TIM_SR_UIF) {
        TIM1_SR = ~TIM_SR_UIF;
        if (TIM1_CCR4) COMP_CSR &= ~(0x700 << COMP_SHIFT);
    }
}

// TIM1 capture compare ISR
void tim1_cc_isr(void) {
    if (TIM1_SR & TIM_SR_CC4IF) {
        TIM1_SR = ~TIM_SR_CC4IF;
        COMP_CSR |= 0x400 << COMP_SHIFT;
    }
}

// DMA1 Channel1 ISR
void dma1_channel1_isr(void) {
    DMA1_IFCR = DMA_IFCR_CTCIF(1);
    DMA1_CCR(1) = 0;
    ADC1_CR2 = ADC_CR2_ADON | ADC_CR2_TSVREFE;
    int r = 4914000 / buf[len - 1];
    adcdata(TEMP_FUNC(buf[len - 2] * r >> 12), buf[0] * r >> 12, buf[1] * r >> 12, buf[2] * r >> 12, buf[3] * r >> 12);
}

