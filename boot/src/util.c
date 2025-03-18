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

#ifndef FLASH_CR_STRT
#define FLASH_CR_STRT FLASH_CR_START
#endif

uint32_t crc32(const char *buf, int len) {
	uint32_t *val = (uint32_t *)buf;
	len >>= 2;
	CRC_CR = CRC_CR_RESET | CRC_CR_REV_IN_WORD | CRC_CR_REV_OUT;
	while (len--) CRC_DR = *val++;
	return ~CRC_DR;
}

__attribute__((__section__(".ramtext")))
int write(char *dst, const char *src, int len) {
	FLASH_KEYR = FLASH_KEYR_KEY1;
	FLASH_KEYR = FLASH_KEYR_KEY2;
	FLASH_SR = -1; // Clear errors
	FLASH_CR = FLASH_CR_PER;
	uint32_t ofs = ((uint32_t)dst + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1); // Align upward to page boundary

	// ------------------- Erase Pages -------------------
#ifdef STM32F0
	for (int pos = 0; pos < len; pos += PAGE_SIZE) {
		FLASH_AR = ofs + pos;
		FLASH_CR = FLASH_CR_PER | FLASH_CR_STRT;
		while (FLASH_SR & FLASH_SR_BSY);
	}
#elif defined(STM32F3)
	for (int pos = 0; pos < len; pos += PAGE_SIZE) {
		FLASH_AR = ofs + pos;
		FLASH_CR = FLASH_CR_PER | FLASH_CR_STRT;
		while (FLASH_SR & FLASH_SR_BSY);
	}
#else
	for (int pos = 0; pos < len; pos += PAGE_SIZE) {
		FLASH_CR = FLASH_CR_PER | FLASH_CR_STRT | ((ofs + pos - (uint32_t)_rom) / PAGE_SIZE) << FLASH_CR_PNB_SHIFT;
		while (FLASH_SR & FLASH_SR_BSY);
	}
#endif

	FLASH_CR = FLASH_CR_PG;

// ------------------- Write Data -------------------
#ifdef STM32F0
	for (int pos = 0; pos < len; pos += 2) {
		*(uint16_t *)(dst + pos) = *(uint16_t *)(src + pos);
#else
	for (int pos = 0; pos < len; pos += 8) {
		*(uint32_t *)(dst + pos) = *(uint32_t *)(src + pos);
		*(uint32_t *)(dst + pos + 4) = *(uint32_t *)(src + pos + 4);
#endif
		// Tunggu sampai operasi selesai
		while (FLASH_SR & FLASH_SR_BSY);
	}

	FLASH_CR = FLASH_CR_LOCK;

// ------------------- Error Checking -------------------
#ifdef STM32F0
	if (FLASH_SR & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)) return 0;
#elif defined(STM32F3)
	if (FLASH_SR & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)) return 0;
#else
	if (FLASH_SR & (FLASH_SR_PROGERR | FLASH_SR_WRPERR)) return 0;
#endif

// ------------------- Verify Written Data -------------------
	for (int pos = 0; pos < len; pos += 4) {
		if (*(uint32_t *)(dst + pos) != *(uint32_t *)(src + pos)) return 0;
	}

	return 1;
}

__attribute__((__section__(".ramtext")))
void update(char *dst, const char *src, int len) {
	if (!write(dst, src, len)) return;
	SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
	for (;;);
}

void setwrp(int type) {
	FLASH_KEYR = FLASH_KEYR_KEY1;
	FLASH_KEYR = FLASH_KEYR_KEY2;
	FLASH_OPTKEYR = FLASH_OPTKEYR_KEY1;
	FLASH_OPTKEYR = FLASH_OPTKEYR_KEY2;
	FLASH_SR = -1; // Clear errors

#ifdef STM32F0
	char opts[6] = {FLASH_OPTION_BYTE_0, FLASH_OPTION_BYTE_1, FLASH_OPTION_BYTE_2, FLASH_OPTION_BYTE_3, 0xff, 0xff};
	if (type == 1) opts[4] = ~((1 << ((_rom_end - _rom + 4095) >> 12)) - 1);
	else if (type == 2) opts[4] = opts[5] = 0;

	FLASH_CR = FLASH_CR_OPTWRE | FLASH_CR_OPTER;
	FLASH_CR = FLASH_CR_OPTWRE | FLASH_CR_OPTER | FLASH_CR_STRT;
	while (FLASH_SR & FLASH_SR_BSY);

	FLASH_CR = FLASH_CR_OPTWRE | FLASH_CR_OPTPG;
	for (int i = 0; i < 6; ++i) {
		FLASH_OPTION_BYTE(i) = opts[i];
		while (FLASH_SR & FLASH_SR_BSY);
	}

	if (FLASH_SR & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)) return;

#elif defined(STM32F3)
    FLASH_WRPR =
        type == 1 ? (((_rom_end - _rom + 2047) >> 11) - 1) << 16:
        type == 2 ? 0xff0000 : 0xff;
	
    	FLASH_CR = FLASH_CR_OPTWRE | FLASH_CR_OPTER;
	FLASH_CR = FLASH_CR_OPTWRE | FLASH_CR_OPTER | FLASH_CR_STRT;
    while (FLASH_SR & FLASH_SR_BSY);
	
    if (FLASH_SR & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)) return;
#else
	FLASH_WRP1AR =
		type == 1 ? (((_rom_end - _rom + 2047) >> 11) - 1) << 16:
		type == 2 ? 0xff0000 : 0xff;
	FLASH_CR = FLASH_CR_OPTSTRT;
	while (FLASH_SR & FLASH_SR_BSY);
	if (FLASH_SR & (FLASH_SR_PROGERR | FLASH_SR_WRPERR)) return;
#endif

#ifdef AT32F4
	SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
#else
	FLASH_CR = FLASH_CR_OBL_LAUNCH;
#endif
	for (;;);
}
