/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2021 Adrian Chadd<ian@freebsd.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include "opt_platform.h"

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/reboot.h>
#include <sys/devmap.h>

#include <vm/vm.h>

#include <machine/bus.h>
#include <machine/intr.h>
#include <machine/machdep.h>
#include <machine/platformvar.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>

#include "platform_if.h"


void qca_msm_early_putc(int c);

/*
 * Early putc routine for EARLY_PRINTF support.  To use, add to kernel config:
 *   option SOCDEV_PA=0x07800000
 *   option SOCDEV_VA=0x07800000
 *   option EARLY_PRINTF
 * Resist the temptation to change the #if 0 to #ifdef EARLY_PRINTF here. It
 * makes sense now, but if multiple SOCs do that it will make early_putc another
 * duplicate symbol to be eliminated on the path to a generic kernel.
 */
#if 1
void
qca_msm_early_putc(int c)
{
	static int is_init = 0;

	int limit;
#define UART_BASE 0xf78af000

	volatile uint32_t * UART_DM_TF0   = (uint32_t *)(UART_BASE + 0x70);
	volatile uint32_t * UART_DM_SR   = (uint32_t *)(UART_BASE + 0x08);
#define UART_DM_SR_TXEMT (1 << 3)
#define UART_DM_SR_TXRDY (1 << 2)
	volatile uint32_t * UART_DM_ISR   = (uint32_t *)(UART_BASE + 0x14);
	volatile uint32_t * UART_DM_CR   = (uint32_t *)(UART_BASE + 0x10);
#define UART_DM_TX_READY (1 << 7)
#define UART_DM_CLEAR_TX_READY 0x300
	volatile uint32_t * UART_DM_NO_CHARS_FOR_TX = (uint32_t *)(UART_BASE + 0x40);
	volatile uint32_t * UART_DM_TFWR = (uint32_t *)(UART_BASE + 0x1c);
#define UART_DM_TFW_VALUE 0
	volatile uint32_t * UART_DM_IPR = (uint32_t *)(UART_BASE + 0x18);
#define  UART_DM_STALE_TIMEOUT_LSB 0xf

	if (is_init == 0) {
		is_init = 1;
		*UART_DM_TFWR = UART_DM_TFW_VALUE;
		wmb();
		*UART_DM_IPR = UART_DM_STALE_TIMEOUT_LSB;
		wmb();
	}
	
	/* Wait until TXFIFO is empty via ISR */
	limit = 100000;
#if 1
	if ((*UART_DM_SR & UART_DM_SR_TXEMT) == 0) {
		while (((*UART_DM_ISR & UART_DM_TX_READY) == 0) && --limit) {
			/* Note - can't use DELAY here yet, too early */
			rmb();
		}
		*UART_DM_CR = UART_DM_CLEAR_TX_READY;
		wmb();
	}
#endif

	/* FIFO is ready.  Say we're going to write one byte */
	*UART_DM_NO_CHARS_FOR_TX = 1;
	wmb();

#if 1
	limit = 100000;
	while (((*UART_DM_SR & UART_DM_SR_TXRDY) == 0) && --limit) {
		/* Note - can't use DELAY here yet, too early */
		rmb();
	}
#endif

	/* Put character in first fifo slot */
	*UART_DM_TF0 = c;
	wmb();
}
early_putc_t *early_putc = qca_msm_early_putc;
#endif
