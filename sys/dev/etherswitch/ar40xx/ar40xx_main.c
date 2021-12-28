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

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/errno.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_arp.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>

#include <machine/bus.h>
#include <dev/iicbus/iic.h>
#include <dev/iicbus/iiconf.h>
#include <dev/iicbus/iicbus.h>
#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include <dev/mdio/mdio.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/etherswitch/etherswitch.h>
#include <dev/etherswitch/ar40xx/ar40xx_var.h>

#include "mdio_if.h"
#include "miibus_if.h"
#include "etherswitch_if.h"

static struct ofw_compat_data compat_data[] = {
	{ "qcom,ess-switch",		1 },
	{ NULL,				0 },
};

static int
ar40xx_probe(device_t dev)
{

//	device_printf(dev, "%s: called\n", __func__);

	if (! ofw_bus_status_okay(dev)) {
//		device_printf(dev, "%s: not bus status ok\n", __func__);
		return (ENXIO);
	}

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0) {
//		device_printf(dev, "%s: didn't find a match\n", __func__);
		return (ENXIO);
	}

	device_set_desc(dev, "IPQ4018 ESS Switch fabric / PSGMII PHY");
	return (BUS_PROBE_DEFAULT);
}

static int
ar40xx_attach(device_t dev)
{
	struct ar40xx_softc *sc = device_get_softc(dev);

	/* sc->sc_switchtype is already decided in ar40xx_probe() */
	sc->sc_dev = dev;
	mtx_init(&sc->sc_mtx, "ar40xx", NULL, MTX_DEF);

	// get switch base address

	// get psgmii base address

	// get switch_mac_mode

	// get clock

	// get reset

	// switch_cpu_bmp

	// switch_lan_bmp

	// switch_wan_bmp

	// .. do we need the ipq4019-mdio node here? for mii-bus? or?

	/*
	 * Ok, at this point we have enough resources to do an initial
	 * reset and configuration.
	 */

	// ess reset

	// psgmii_self_test

	// psgmii_self_test_clean

	// mac_mode_init

	// init_port for each port

	// init_globals

	return (0);
}

static int
ar40xx_detach(device_t dev)
{
	struct ar40xx_softc *sc = device_get_softc(dev);

	mtx_destroy(&sc->sc_mtx);

	return (0);
}

static device_method_t ar40xx_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		ar40xx_probe),
	DEVMETHOD(device_attach,	ar40xx_attach),
	DEVMETHOD(device_detach,	ar40xx_detach),

	/* bus interface */
	DEVMETHOD(bus_add_child,	device_add_child_ordered),

	DEVMETHOD_END
};

DEFINE_CLASS_0(ar40xx, ar40xx_driver, ar40xx_methods,
    sizeof(struct ar40xx_softc));
static devclass_t ar40xx_devclass;

DRIVER_MODULE(ar40xx, simplebus, ar40xx_driver, ar40xx_devclass, 0, 0);
DRIVER_MODULE(ar40xx, ofwbus, ar40xx_driver, ar40xx_devclass, 0, 0);

// TODO: yes, we need to get the rest of the dependencies in here

#if 0
DRIVER_MODULE(ar40xx, mdio, ar40xx_driver, ar40xx_devclass, 0, 0);
DRIVER_MODULE(miibus, ar40xx, miibus_driver, miibus_devclass, 0, 0);
DRIVER_MODULE(mdio, ar40xx, mdio_driver, mdio_devclass, 0, 0);
DRIVER_MODULE(etherswitch, ar40xx, etherswitch_driver, etherswitch_devclass, 0, 0);
MODULE_VERSION(ar40xx, 1);
MODULE_DEPEND(ar40xx, mdio, 1, 1, 1);
#endif

//MODULE_DEPEND(ar40xx, miibus, 1, 1, 1); /* XXX which versions? */
//MODULE_DEPEND(ar40xx, etherswitch, 1, 1, 1); /* XXX which versions? */
