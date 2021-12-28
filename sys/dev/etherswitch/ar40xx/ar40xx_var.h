/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2021 Adrian Chadd <adrian@FreeBSD.org>.
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
 *
 * $FreeBSD$
 */
#ifndef	__AR40XX_VAR_H__
#define	__AR40XX_VAR_H__


struct ar40xx_softc {
	struct mtx	sc_mtx;		/* serialize access to softc */
	device_t	sc_dev;

	/* ess-switch memory resource */
	struct resource	*sc_ess_mem_res;
	int		sc_ess_mem_rid;

	/* ess-switch clock resource */
	clk_t		sc_ess_clk;

	/* ess-switch reset resource */
	hwreset_t	sc_ess_rst;

	/* memory for the ess-psgmii config interface */
	bus_space_tag_t		sc_psgmii_mem_tag;
	bus_space_handle_t	sc_psgmii_mem_handle;
	bus_size_t		sc_psgmii_mem_size;

	/* reference to the ipq4019-mdio interface */
	phandle_t		sc_mdio_phandle;

	struct {
		uint32_t switch_mac_mode;
		uint32_t switch_cpu_bmp;
		uint32_t switch_lan_bmp;
		uint32_t switch_wan_bmp;
	} sc_config;
};

#endif	/* __AR40XX_VAR_H__ */

