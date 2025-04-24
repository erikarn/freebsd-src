/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2025 Adrian Chadd <adrian@FreeBSD.org>
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "opt_inet.h"
#include "opt_inet6.h"
#include "opt_rss.h"

#include "ixgbe.h"
#include "mdio_if.h"
#include "ixgbe_sriov.h"
#include "ifdi_if.h"
#include "ixgbe_mdio.h"
#include "if_ix_mdio.h"

#include <dev/mdio/mdio.h>

int
ixgbe_mdio_readreg(device_t dev, int phy, int reg)
{
	if_ctx_t ctx = device_get_softc(dev);
	struct ixgbe_softc *sc = iflib_get_softc(ctx);
	struct ixgbe_hw *hw = &sc->hw;
	uint16_t val = 0;
	int32_t ret = 0;

	ret = ixgbe_read_mdio_c22(hw, phy, reg, &val);

	if (ret != IXGBE_SUCCESS) {
		device_printf(dev, "%s: read_mdi_22 failed (%d)\n",
		    __func__, ret);
		return (-1);
	}
	return (val);
}

int
ixgbe_mdio_writereg(device_t dev, int phy, int reg, int data)
{
	if_ctx_t ctx = device_get_softc(dev);
	struct ixgbe_softc *sc = iflib_get_softc(ctx);
	struct ixgbe_hw *hw = &sc->hw;
	int32_t ret;

	ret = ixgbe_write_mdio_c22(hw, phy, reg, data);
	if (ret != IXGBE_SUCCESS) {
		device_printf(dev, "%s: write_mdi_22 failed (%d)\n",
		    __func__, ret);
		return (-1);
	}
	return (0);
}

void
ixgbe_mdio_attach(struct ixgbe_softc *sc)
{

	device_add_child(sc->dev, "mdio", DEVICE_UNIT_ANY);
	bus_attach_children(sc->dev);
}
