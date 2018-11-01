/*****************************************************************************
 *         S I M P L I F I E D   X M O D E M   P R O T O C O L
 *
 *
 *                                 v 1.0
 *
 * xmod.c - basic simplified implementation of the xmodem/crc protocol
 * Copyright (C) 2018 Andreas J. Reichel

 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA. 
 *
 *
 *                 for use with typical Atmel UART libs
 *               needs uart_putc, uart_available, uart_getc
 *
 *     define a buffer in your application of type uint8_t[XMOD_BUFFSIZE];
 *
 *       This file only supports the CRC version, not the CHKSUM version.
 *     The provided buffer is used circularly and must be a multiple of 128.
 *
 *****************************************************************************/

#define _GNU_SOURCE
#include <stdint.h>
#include <stdbool.h>

#ifndef XMOD_BUFFSIZE
#define XMOD_BUFFSIZE	256
#endif

extern uint16_t GetMillis();
extern void ResetMillis();

extern void uart_putc(char c);
extern bool uart_available();
extern char uart_getc();

extern uint8_t buffer[XMOD_BUFFSIZE];

#ifdef DEBUG
#include <stdio.h>
#define DEBUG_PRINT(a, ...) fprintf(stdout, a __VA_OPT__(,) __VA_ARGS__);
#else
#define DEBUG_PRINT(a, ...) {}
#endif

#define MAXRETRY	15
#define TIMEOUT_MS	1000

#define X_SOH		1
#define X_EOT		4
#define X_ACK		6
#define X_NAK		21
#define X_ETB		23
#define X_CAN		24

typedef enum {
	TRANS_AWAIT,
	TRANS_RUN,
	TRANS_ABORT,
	TRANS_BLOCK,
	TRANS_BLOCK2,
	TRANS_CRC1,
	TRANS_CRC2,
	TRANS_END
} STATE;

static STATE state;

static int retrycount;


static int counter;
static uint16_t current_crc;
static uint16_t crc_recv;
static uint16_t mempos;
static uint8_t blocknum;
static uint8_t last_succeeded_block;

#define POLY 0x1021 

uint16_t xmod_crc_update(uint16_t CRC_acc, uint8_t CRC_input)
{
   	CRC_acc ^= ((uint16_t) CRC_input << 8); 
   	for (uint8_t i = 0; i < 8; i++) {
		if (CRC_acc & 0x8000) { 
		       	CRC_acc <<= 1;
       			CRC_acc ^= POLY; 
       		} else CRC_acc <<= 1; 
     	} 
   	return CRC_acc; 
} 

STATE xmod_sendblock()
{
	uart_putc(X_SOH);
	uart_putc(blocknum);
	uart_putc(0xFF - blocknum);
	current_crc = 0;
	uint8_t byte;
	uint16_t offset = 128*(blocknum-1);
	if (offset >= XMOD_BUFFSIZE) {
		offset = 0;
	}
	for (uint8_t p = 0; p < 128; p++) {
		byte = buffer[(p + offset)];
		current_crc = xmod_crc_update(current_crc, byte);
		uart_putc(byte);
	}
	DEBUG_PRINT("%04X\n", current_crc);
	uart_putc((current_crc >> 8) & 0xFF);
	uart_putc(current_crc & 0xFF);
}

STATE handlebyte(char d)
{
	if (state == TRANS_AWAIT && d == X_SOH) {
		counter = 0;
		current_crc = 0;
		return TRANS_BLOCK;
	} else
	if (state == TRANS_BLOCK) {
		blocknum = (uint8_t) d;
		return TRANS_BLOCK2;
	} else
	if (state == TRANS_BLOCK2) {
		if (blocknum != 0xFF - (uint8_t) d) {
			DEBUG_PRINT(" Invalid block number\n");
			return TRANS_ABORT;
		}
		if (blocknum == last_succeeded_block) {
			mempos -= 128;
			DEBUG_PRINT(" Block %02d sent twice, ignoring\n", blocknum);
		}
		return TRANS_RUN;
	} else
	if (state == TRANS_AWAIT && d == X_EOT) {
		DEBUG_PRINT(" End of transmission\n");
		uart_putc(X_ACK);
		uart_putc(X_ACK);
		return TRANS_END;
	} else
	if (state == TRANS_AWAIT && d == X_ETB) {
		uart_putc(X_ACK);
		return TRANS_END;
	} else if (state == TRANS_AWAIT) {
		DEBUG_PRINT(" Got invalid byte: %02X", d);
		return TRANS_ABORT;
	} else if (state == TRANS_RUN) {
		counter++;
		current_crc = xmod_crc_update(current_crc, d);
		buffer[mempos++] = d;
		if (counter == 128) {
			return TRANS_CRC1;
		}	
	} else if (state == TRANS_CRC1) {
		crc_recv = (uint16_t) d << 8;
		return TRANS_CRC2;
	} else if (state == TRANS_CRC2) {
		crc_recv |= (uint8_t) d;
		if (crc_recv == current_crc) {
			uart_putc(X_ACK);
			last_succeeded_block = blocknum;
		} else
		{
			DEBUG_PRINT("Invalid CRC, NAK\n");
			uart_putc(X_NAK);
			mempos -= 128;
		}
		return TRANS_AWAIT;
	}
	return TRANS_RUN;
}

void xmod_upload()
{
	state = TRANS_AWAIT;
	blocknum = 1;
	last_succeeded_block = 0;

	char d;
	do {
		if (!uart_available()) {
			if (GetMillis() <= TIMEOUT_MS) continue;
			if (retrycount > MAXRETRY) {
				state = TRANS_ABORT;
				continue;
			}
			retrycount++;
			ResetMillis();
		} else {
			ResetMillis();
			d = uart_getc();
			if (d == 'C') {
				blocknum = 1;
				state = xmod_sendblock();
				continue;
			} else
			if (d == X_ACK) {
				blocknum++;
				if (blocknum == (XMOD_BUFFSIZE / 128 + 1)) {
					uart_putc(X_EOT);
					state = TRANS_END;
					continue;
				}				
			}
			xmod_sendblock();
		}
	} while (state != TRANS_END && state != TRANS_ABORT);
}

void xmod_download()
{
	mempos = 0;
	retrycount = 0;
	last_succeeded_block = 0;
	
	uart_putc('C');
	state = TRANS_AWAIT;
	do {
		if (!uart_available()) {
			if (GetMillis() <= TIMEOUT_MS) continue;
			if (retrycount > MAXRETRY) {
				state = TRANS_ABORT;
				continue;
			}
			uart_putc('C');
			retrycount++;
		} else {
			state = handlebyte(uart_getc());
		}
		ResetMillis();
	} while(state != TRANS_ABORT && state != TRANS_END);
	while (uart_available()) {
		(void)uart_getc(); // eat up garbage
	}
}
