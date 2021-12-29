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
#include <dev/extres/clk/clk.h>
#include <dev/extres/hwreset/hwreset.h>

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

	if (! ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "IPQ4018 ESS Switch fabric / PSGMII PHY");
	return (BUS_PROBE_DEFAULT);
}

static int
ar40xx_attach(device_t dev)
{
	struct ar40xx_softc *sc = device_get_softc(dev);
	phandle_t psgmii_p, root_p, mdio_p;
	int ret;

	sc->sc_dev = dev;
	mtx_init(&sc->sc_mtx, "ar40xx", NULL, MTX_DEF);

	psgmii_p = OF_finddevice("/soc/ess-psgmii");
	if (psgmii_p == -1) {
		device_printf(dev,
		    "%s: couldn't find /soc/ess-psgmii DT node\n",
		    __func__);
		goto error;
	}

	// get the ipq4019-mdio node here, to talk to our local PHYs if needed
	root_p = OF_finddevice("/soc");
	mdio_p = ofw_bus_find_compatible(root_p, "qcom,ipq4019-mdio");
	if (mdio_p == -1) {
		device_printf(dev, "%s: couldn't find ipq4019-mdio DT node\n",
		    __func__);
		goto error;
	}
	sc->sc_mdio_phandle = mdio_p;

	// get psgmii base address from psgmii node
	ret = OF_decode_addr(psgmii_p, 0, &sc->sc_psgmii_mem_tag,
	    &sc->sc_psgmii_mem_handle,
	    &sc->sc_psgmii_mem_size);
	if (ret != 0) {
		device_printf(dev, "%s: couldn't map psgmii mem (%d)\n",
		    __func__, ret);
		goto error;
	}

	// get switch base address
	sc->sc_ess_mem_rid = 0;
	sc->sc_ess_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->sc_ess_mem_rid, RF_ACTIVE);
	if (sc->sc_ess_mem_res == NULL) {
		device_printf(dev, "%s: failed to find memory resource\n",
		    __func__);
		goto error;
	}

	// get switch_mac_mode
	ret = OF_getencprop(ofw_bus_get_node(dev), "switch_mac_mode",
	    &sc->sc_config.switch_mac_mode,
	    sizeof(sc->sc_config.switch_mac_mode));
	if (ret < 0) {
		device_printf(dev, "%s: missing switch_mac_mode property\n",
		    __func__);
		goto error;
	}

	// switch_cpu_bmp
	ret = OF_getencprop(ofw_bus_get_node(dev), "switch_cpu_bmp",
	    &sc->sc_config.switch_cpu_bmp,
	    sizeof(sc->sc_config.switch_cpu_bmp));
	if (ret < 0) {
		device_printf(dev, "%s: missing switch_cpu_bmp property\n",
		    __func__);
		goto error;
	}

	// switch_lan_bmp
	ret = OF_getencprop(ofw_bus_get_node(dev), "switch_lan_bmp",
	    &sc->sc_config.switch_lan_bmp,
	    sizeof(sc->sc_config.switch_lan_bmp));
	if (ret < 0) {
		device_printf(dev, "%s: missing switch_lan_bmp property\n",
		    __func__);
		goto error;
	}

	// switch_wan_bmp
	ret = OF_getencprop(ofw_bus_get_node(dev), "switch_wan_bmp",
	    &sc->sc_config.switch_wan_bmp,
	    sizeof(sc->sc_config.switch_wan_bmp));
	if (ret < 0) {
		device_printf(dev, "%s: missing switch_wan_bmp property\n",
		    __func__);
		goto error;
	}

	// get clock
	ret = clk_get_by_ofw_name(dev, 0, "ess_clk", &sc->sc_ess_clk);
	if (ret != 0) {
		device_printf(dev, "%s: failed to find ess_clk (%d)\n",
		    __func__, ret);
		goto error;
	}

	// get reset
	ret = hwreset_get_by_ofw_name(dev, 0, "ess_rst", &sc->sc_ess_rst);
	if (ret != 0) {
		device_printf(dev, "%s: failed to find ess_rst (%d)\n",
		    __func__, ret);
		goto error;
	}


	/*
	 * Ok, at this point we have enough resources to do an initial
	 * reset and configuration.
	 */
	device_printf(dev, "%s: TODO\n", __func__);

	// ess reset

	// psgmii_self_test

	// psgmii_self_test_clean

	// mac_mode_init

	// init_port for each port

	// init_globals

	// sw reset switch

	// cpuport setup

	// start qm error check

	return (0);
error:
	/* XXX TODO: free resources */
	return (ENXIO);
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
MODULE_VERSION(ar40xx, 1);

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
