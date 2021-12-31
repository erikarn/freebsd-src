/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2021 Adrian Chadd <adrian@FreeBSD.org>
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>

#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/gpio.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <dev/gpio/gpiobusvar.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/qcom_ess_edma/qcom_ess_edma_var.h>
#include <dev/qcom_ess_edma/qcom_ess_edma_reg.h>
#include <dev/qcom_ess_edma/qcom_ess_edma_hw.h>

/*
 * Reset the ESS EDMA core.
 *
 * This is ... problematic.  There's only a single clock control
 * for the ESS core - and that includes both the EDMA (ethernet)
 * and switch hardware.
 *
 * AND, it's a placeholder for what the linux ess-edma driver
 * is doing directly to the ess core because in some instances
 * where there's a single PHY hooked up, it's possible that
 * ess-switch won't be initialised.  In that case it defaults
 * to a very minimal switch config.  Now, that's honestly pretty
 * bad, and instead we should be doing that kind of awareness
 * in ar40xx_switch.
 *
 * So, for now this is a big no-op, at least until everything
 * is implemented enough that I can get the switch/phy code and
 * this EDMA driver code to co-exist.
 */
int
qcom_ess_edma_hw_reset(struct qcom_ess_edma_softc *sc)
{

	EDMA_LOCK_ASSERT(sc);

	device_printf(sc->sc_dev, "%s: called, TODO!\n", __func__);

	/*
	 * This is where the linux ess-edma driver would reset the
	 * ESS core.
	 */

	/*
	 * and here's where the linux ess-edma driver would program
	 * in the initial port config, rgmii control, traffic
	 * port forwarding and broadcast/multicast traffic forwarding.
	 *
	 * instead, this should be done by the ar40xx_switch driver!
	 */
	return (0);
}

/*
 * Set the TX interrupt moderation timer.
 *
 * The resolution of this register is 2uS.
 */
int
qcom_ess_edma_hw_set_tx_intr_moderation(struct qcom_ess_edma_softc *sc,
    uint32_t usec)
{
	uint32_t reg;

	EDMA_LOCK_ASSERT(sc);

	EDMA_REG_BARRIER_READ(sc);
	reg = EDMA_REG_READ(sc, EDMA_REG_IRQ_MODRT_TIMER_INIT);
	reg &= ~(EDMA_IRQ_MODRT_TIMER_MASK << EDMA_IRQ_MODRT_TX_TIMER_SHIFT);
	reg |= (usec & EDMA_IRQ_MODRT_TIMER_MASK)
	    << EDMA_IRQ_MODRT_TX_TIMER_SHIFT;
	EDMA_REG_WRITE(sc, EDMA_REG_IRQ_MODRT_TIMER_INIT, reg);
	EDMA_REG_BARRIER_WRITE(sc);

	return (0);
}

/*
 * Set the RX interrupt moderation timer.
 *
 * The resolution of this register is 2uS.
 */
int
qcom_ess_edma_hw_set_rx_intr_moderation(struct qcom_ess_edma_softc *sc,
    uint32_t usec)
{
	uint32_t reg;

	EDMA_LOCK_ASSERT(sc);

	EDMA_REG_BARRIER_READ(sc);
	reg = EDMA_REG_READ(sc, EDMA_REG_IRQ_MODRT_TIMER_INIT);
	reg &= ~(EDMA_IRQ_MODRT_TIMER_MASK << EDMA_IRQ_MODRT_RX_TIMER_SHIFT);
	reg |= (usec & EDMA_IRQ_MODRT_TIMER_MASK)
	    << EDMA_IRQ_MODRT_RX_TIMER_SHIFT;
	EDMA_REG_WRITE(sc, EDMA_REG_IRQ_MODRT_TIMER_INIT, reg);
	EDMA_REG_BARRIER_WRITE(sc);

	return (0);
}

/*
 * Disable all interrupts.
 */
int
qcom_ess_edma_hw_intr_disable(struct qcom_ess_edma_softc *sc)
{
	int i;

	/* Disable TX interrupts */
	for (i = 0; i < QCOM_ESS_EDMA_NUM_TX_IRQS; i++) {
		EDMA_REG_WRITE(sc, EDMA_REG_TX_INT_MASK_Q(i), 0);
	}

	/* Disable RX interrupts */
	for (i = 0; i < QCOM_ESS_EDMA_NUM_RX_IRQS; i++) {
		EDMA_REG_WRITE(sc, EDMA_REG_RX_INT_MASK_Q(i), 0);
	}

	/* Disable misc/WOL interrupts */
	EDMA_REG_WRITE(sc, EDMA_REG_MISC_IMR, 0);
	EDMA_REG_WRITE(sc, EDMA_REG_WOL_IMR, 0);

	EDMA_REG_BARRIER_WRITE(sc);

	return (0);
}

/*
 * Enable interrupts.
 */
int
qcom_ess_edma_hw_intr_enable(struct qcom_ess_edma_softc *sc)
{
	int i;

	/* ACK, then Enable TX interrupts */
	EDMA_REG_WRITE(sc, EDMA_REG_TX_ISR, 0xffff);
	for (i = 0; i < QCOM_ESS_EDMA_NUM_TX_IRQS; i++) {
		EDMA_REG_WRITE(sc, EDMA_REG_TX_INT_MASK_Q(i),
		    sc->sc_config.tx_intr_mask);
	}

	/* ACK, then Enable RX interrupts */
	EDMA_REG_WRITE(sc, EDMA_REG_RX_ISR, 0xff);
	for (i = 0; i < QCOM_ESS_EDMA_NUM_RX_IRQS; i++) {
		EDMA_REG_WRITE(sc, EDMA_REG_RX_INT_MASK_Q(i),
		    sc->sc_config.rx_intr_mask);
	}

	/* Disable misc/WOL interrupts */
	EDMA_REG_WRITE(sc, EDMA_REG_MISC_IMR, 0);
	EDMA_REG_WRITE(sc, EDMA_REG_WOL_IMR, 0);

	EDMA_REG_BARRIER_WRITE(sc);

	return (0);
}

/*
 * Clear interrupt status.
 */
int
qcom_ess_edma_hw_intr_status_clear(struct qcom_ess_edma_softc *sc)
{

	EDMA_REG_WRITE(sc, EDMA_REG_RX_ISR, 0xff);
	EDMA_REG_WRITE(sc, EDMA_REG_TX_ISR, 0xffff);
	EDMA_REG_WRITE(sc, EDMA_REG_MISC_ISR, 0x1fff);
	EDMA_REG_WRITE(sc, EDMA_REG_WOL_ISR, 0x1);

	return (0);
}

/*
 * Configure the default RSS indirection table.
 */
int
qcom_ess_edma_hw_configure_rss_table(struct qcom_ess_edma_softc *sc)
{
	int i;

	/*
	 * The default IDT value configures the hash buckets
	 * to a repeating pattern of q0, q2, q4, q6.
	 */
	for (i = 0; i < EDMA_NUM_IDT; i++) {
		EDMA_REG_WRITE(sc, EDMA_REG_RSS_IDT(i), EDMA_RSS_IDT_VALUE);
	}
	EDMA_REG_BARRIER_WRITE(sc);

	return (0);
}

/*
 * Configure the default load balance mapping table.
 */
int
qcom_ess_edma_hw_configure_load_balance_table(struct qcom_ess_edma_softc *sc)
{

	/*
	 * I think this is mapping things to queues 0,2,4,6.
	 * Linux says it's 0,1,3,4 but that doesn't match the
	 * EDMA_LB_REG_VALUE field.
	 */
	EDMA_REG_WRITE(sc, EDMA_REG_LB_RING, EDMA_LB_REG_VALUE);
	EDMA_REG_BARRIER_WRITE(sc);
	return (0);
}

/*
 * Configure the default virtual tx ring queues.
 */
int
qcom_ess_edma_hw_configure_tx_virtual_queue(struct qcom_ess_edma_softc *sc)
{

	EDMA_REG_WRITE(sc, EDMA_REG_VQ_CTRL0, EDMA_VQ_REG_VALUE);
	EDMA_REG_WRITE(sc, EDMA_REG_VQ_CTRL1, EDMA_VQ_REG_VALUE);

	EDMA_REG_BARRIER_WRITE(sc);
	return (0);
}

/*
 * Configure the default maximum AXI bus transaction size.
 */
int
qcom_ess_edma_hw_configure_default_axi_transaction_size(
    struct qcom_ess_edma_softc *sc)
{

	EDMA_REG_WRITE(sc, EDMA_REG_AXIW_CTRL_MAXWRSIZE,
	    EDMA_AXIW_MAXWRSIZE_VALUE);
	return (0);
}

/*
 * Stop the TX/RX queues.
 */
int
qcom_ess_edma_hw_stop_txrx_queues(struct qcom_ess_edma_softc *sc)
{
	uint32_t reg;

	EDMA_REG_BARRIER_READ(sc);
	reg = EDMA_REG_READ(sc, EDMA_REG_RXQ_CTRL);
	reg &= ~EDMA_RXQ_CTRL_EN;
	EDMA_REG_WRITE(sc, EDMA_REG_RXQ_CTRL, reg);
	EDMA_REG_BARRIER_WRITE(sc);

	EDMA_REG_BARRIER_READ(sc);
	reg = EDMA_REG_READ(sc, EDMA_REG_TXQ_CTRL);
	reg &= ~EDMA_TXQ_CTRL_TXQ_EN;
	EDMA_REG_WRITE(sc, EDMA_REG_TXQ_CTRL, reg);
	EDMA_REG_BARRIER_WRITE(sc);
	return (0);
}

/*
 * Stop the EDMA block, disable interrupts.
 */
int
qcom_ess_edma_hw_stop(struct qcom_ess_edma_softc *sc)
{
	int ret;

	EDMA_LOCK_ASSERT(sc);

	ret = qcom_ess_edma_hw_intr_disable(sc);
	ret = qcom_ess_edma_hw_intr_status_clear(sc);
	ret = qcom_ess_edma_hw_stop_txrx_queues(sc);

	return (0);
}

/*
 * Update the producer index for the given receive queue.
 */
int
qcom_ess_hw_rfd_prod_index_update(struct qcom_ess_edma_softc *sc, int queue,
    int idx)
{
	uint32_t reg;

	EDMA_LOCK_ASSERT(sc);

	EDMA_REG_BARRIER_READ(sc);
	reg = EDMA_REG_READ(sc, EDMA_REG_RFD_IDX_Q(queue));
	reg &= ~EDMA_RFD_PROD_IDX_BITS;
	reg |= idx;
	EDMA_REG_WRITE(sc, EDMA_REG_RFD_IDX_Q(queue), reg);
	EDMA_REG_BARRIER_WRITE(sc);

	return (0);
}

/*
 * Fetch the consumer index for the given receive queue.
 */
int
qcom_ess_hw_rfd_get_cons_index(struct qcom_ess_edma_softc *sc, int queue)
{
	uint32_t reg;

	EDMA_LOCK_ASSERT(sc);

	EDMA_REG_BARRIER_READ(sc);
	reg = EDMA_REG_READ(sc, EDMA_REG_RFD_IDX_Q(queue));
	return (reg >> EDMA_RFD_CONS_IDX_SHIFT) & EDMA_RFD_CONS_IDX_MASK;
}

/*
 * Update the software consumed index to the hardware, so
 * it knows what we've read.
 */
int
qcom_ess_hw_rfd_sw_cons_index_update(struct qcom_ess_edma_softc *sc,
    int queue, int idx)
{
	EDMA_LOCK_ASSERT(sc);

	EDMA_REG_WRITE(sc, EDMA_REG_RX_SW_CONS_IDX_Q(queue), idx);
	EDMA_REG_BARRIER_WRITE(sc);

	return (0);
}
