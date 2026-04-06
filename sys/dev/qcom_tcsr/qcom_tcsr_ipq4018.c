/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2021, Adrian Chadd <adrian@FreeBSD.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
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

#include <sys/param.h>
#include <sys/systm.h>

#include <sys/bus.h>
#include <sys/interrupt.h>
#include <sys/malloc.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/gpio.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_extern.h>

#include <machine/bus.h>
#include <machine/cpu.h>

#include <dev/fdt/fdt_common.h>
#include <dev/fdt/fdt_pinctrl.h>

#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/qcom_tcsr/qcom_tcsr_var.h>
#include <dev/qcom_tcsr/qcom_tcsr_reg.h>

int
qcom_tcsr_init_ipq4018(struct qcom_tcsr_softc *sc)
{
	device_t dev = sc->sc_dev;
	uint32_t val;

	/*
	 * Parse out the open firmware entries to see which particular
	 * configurations we need to set here.
	 */

	/*
	 * USB control select.
	 *
	 * For linux-msm on the IPQ401x, it actually calls into the SCM
	 * to make the change.  OpenWRT just does a register write.
	 * We'll do the register write for now.
	 */
	if (OF_getencprop(ofw_bus_get_node(dev), "qcom,usb-ctrl-select",
	    &val, sizeof(val)) > 0) {
		if (bootverbose)
			device_printf(sc->sc_dev,
			    "USB control select (val 0x%x)\n",
			    val);
		QCOM_TCSR_WRITE_4(sc, QCOM_TCSR_USB_PORT_SEL, val);
	}

	/*
	 * USB high speed phy mode select.
	 */
	if (OF_getencprop(ofw_bus_get_node(dev), "qcom,usb-hsphy-mode-select",
	    &val, sizeof(val)) > 0) {
		if (bootverbose)
			device_printf(sc->sc_dev,
			    "USB high speed PHY mode select (val 0x%x)\n",
			    val);
		QCOM_TCSR_WRITE_4(sc, QCOM_TCSR_USB_HSPHY_CONFIG, val);
	}

	/*
	 * Ethernet switch subsystem interface type select.
	 */
	if (OF_getencprop(ofw_bus_get_node(dev), "qcom,ess-interface-select",
	    &val, sizeof(val)) > 0) {
		uint32_t reg;

		if (bootverbose)
			device_printf(sc->sc_dev,
			    "ESS external interface select (val 0x%x)\n",
			    val);
		reg = QCOM_TCSR_READ_4(sc, QCOM_TCSR_ESS_INTERFACE_SEL_OFFSET);
		reg &= ~QCOM_TCSR_ESS_INTERFACE_SEL_MASK;
		reg |= (val & QCOM_TCSR_ESS_INTERFACE_SEL_MASK);
		QCOM_TCSR_WRITE_4(sc, QCOM_TCSR_ESS_INTERFACE_SEL_OFFSET, reg);
	}

	/*
	 * WiFi GLB select.
	 */
	if (OF_getencprop(ofw_bus_get_node(dev), "qcom,wifi_glb_cfg",
	    &val, sizeof(val)) > 0) {
		if (bootverbose)
			device_printf(sc->sc_dev,
			    "WIFI GLB select (val 0x%x)\n",
			    val);
		QCOM_TCSR_WRITE_4(sc, QCOM_TCSR_WIFI0_GLB_CFG_OFFSET, val);
		QCOM_TCSR_WRITE_4(sc, QCOM_TCSR_WIFI1_GLB_CFG_OFFSET, val);
	}

	/*
	 * WiFi NOC interconnect memory type.
	 */
	if (OF_getencprop(ofw_bus_get_node(dev),
	    "qcom,wifi_noc_memtype_m0_m2",
	    &val, sizeof(val)) > 0) {
		if (bootverbose)
			device_printf(sc->sc_dev,
			    "WiFi NOC memory type (val 0x%x)\n",
			    val);
		QCOM_TCSR_WRITE_4(sc, QCOM_TCSR_PNOC_SNOC_MEMTYPE_M0_M2, val);
	}

	return (0);
}
