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
#include <sys/mbuf.h>
#include <sys/endian.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/qcom_ess_edma/qcom_ess_edma_var.h>
#include <dev/qcom_ess_edma/qcom_ess_edma_reg.h>
#include <dev/qcom_ess_edma/qcom_ess_edma_hw.h>
#include <dev/qcom_ess_edma/qcom_ess_edma_desc.h>
#include <dev/qcom_ess_edma/qcom_ess_edma_rx.h>

/*
 * Allocate a receive buffer for the given ring/index, setup DMA.
 *
 * The caller must have called the ring prewrite routine in order
 * to flush the ring memory if needed before writing to it.
 * It's not done here so we don't do it on /every/ ring update.
 *
 * Returns an error if the slot is full or unable to fill it;
 * the caller should then figure out how to cope.
 */
int
qcom_ess_edma_rx_buf_alloc(struct qcom_ess_edma_softc *sc,
    struct qcom_ess_edma_desc_ring *ring, int idx)
{
	struct mbuf *m;
	struct qcom_ess_edma_sw_desc_rx *rxd;
	struct qcom_ess_edma_rx_free_desc *ds;
	bus_dma_segment_t segs[1];
	int error;
	int nsegs;

	/* Get the software/hardware descriptors we're going to update */
	rxd = qcom_ess_edma_desc_ring_get_sw_desc(sc, ring, idx);
	if (rxd == NULL) {
		device_printf(sc->sc_dev,
		    "ERROR; couldn't get sw desc (idx %d)\n", idx);
		return (EINVAL);
	}
	ds = qcom_ess_edma_desc_ring_get_hw_desc(sc, ring, idx);
	if (rxd == NULL) {
		device_printf(sc->sc_dev,
		    "ERROR; couldn't get hw desc (idx %d)\n", idx);
		return (EINVAL);
	}

	/* If this ring has an mbuf already then return error */
	if (rxd->m != NULL) {
		device_printf(sc->sc_dev,
		    "ERROR: sw desc idx %d already has an mbuf\n",
		    idx);
		return (EINVAL); /* XXX */
	}

	/* Allocate mbuf */
	m = m_get2(sc->sc_config.rx_buf_size, M_NOWAIT, MT_DATA, M_PKTHDR);
	if (m == NULL) {
		/* XXX keep statistics */
		device_printf(sc->sc_dev, "ERROR: failed to allocate mbuf\n");
		return (ENOMEM);
	}

	/* Load dma map, get physical memory address of mbuf */
	nsegs = 1;
	error = bus_dmamap_load_mbuf_sg(ring->buffer_dma_tag, rxd->m_dmamap,
	    m, segs, &nsegs, 0);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "ERROR: couldn't load mbuf dmamap (%d)\n", error);
		m_free(m);
		return (error);
	}

	/* Populate sw and hw desc */
	rxd->m = m;
	rxd->m_physaddr = segs[0].ds_addr;

	ds->addr = htole32(segs[0].ds_addr);

	return (0);
}

/*
 * Remove a receive buffer from the given ring/index.
 *
 * This clears the software/hardware index and unmaps the mbuf;
 * the returned mbuf will be owned by the caller.
 */
struct mbuf *
qcom_ess_edma_rx_buf_clean(struct qcom_ess_edma_softc *sc,
    struct qcom_ess_edma_desc_ring *ring, int idx)
{
	struct mbuf *m;
	struct qcom_ess_edma_sw_desc_rx *rxd;
	struct qcom_ess_edma_rx_free_desc *ds;

	/* Get the software/hardware descriptors we're going to update */
	rxd = qcom_ess_edma_desc_ring_get_sw_desc(sc, ring, idx);
	if (rxd == NULL) {
		device_printf(sc->sc_dev,
		    "ERROR; couldn't get sw desc (idx %d)\n", idx);
		return (NULL);
	}
	ds = qcom_ess_edma_desc_ring_get_hw_desc(sc, ring, idx);
	if (rxd == NULL) {
		device_printf(sc->sc_dev,
		    "ERROR; couldn't get hw desc (idx %d)\n", idx);
		return (NULL);
	}

	/* No mbuf? return null; it's fine */
	if (rxd->m == NULL) {
		return (NULL);
	}

	/* Flush mbuf */
	bus_dmamap_sync(ring->buffer_dma_tag, rxd->m_dmamap,
	    BUS_DMASYNC_POSTREAD);

	/* Unload */
	bus_dmamap_unload(ring->buffer_dma_tag, rxd->m_dmamap);

	/* Remove sw/hw desc entries */
	m = rxd->m;
	rxd->m = NULL;
	/*
	 * XXX Note: removing hw entries is purely for correctness; it may be
	 * VERY SLOW!  Once this is working it should just be removed.
	 */
	ds->addr = 0;

	return (m);
}

/*
 * Fill the current ring, up to 'num' entries (or the ring is full.)
 * It will also update the producer index for the given queue.
 *
 * Returns 0 if OK, error if there's a problem.
 */
int
qcom_ess_edma_rx_ring_fill(struct qcom_ess_edma_softc *sc,
    int queue, int num)
{
	struct qcom_ess_edma_desc_ring *ring;
	int num_fill;
	int idx;
	int error;
	int prod_index;
	int n = 0;

	EDMA_LOCK_ASSERT(sc);

	ring = &sc->sc_rx_ring[queue];

	num_fill = num;
	if (num_fill > ring->ring_count)
		num_fill = ring->ring_count - 1;
	idx = ring->next_to_fill;

	while (num_fill != 0) {
		error = qcom_ess_edma_rx_buf_alloc(sc, ring, idx);
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "ERROR: queue %d: failed to alloc rx buf (%d)\n",
			    queue, error);
			break;
		}
		num_fill--;

		/* Update ring index, wrap at ring_count */
		idx++;
		if (idx >= ring->ring_count)
			idx = 0;
		n++;
	}

	ring->next_to_fill = idx;

	/* Flush ring updates before HW index is updated */
	qcom_ess_edma_desc_ring_flush_preupdate(sc, ring);

	/* producer index is the ring number, minus 1 (ie the slot BEFORE) */
	if (idx == 0)
		prod_index = ring->ring_count - 1;
	else
		prod_index = idx - 1;
	(void) qcom_ess_edma_hw_rfd_prod_index_update(sc, queue, prod_index);

	device_printf(sc->sc_dev, "%s: queue %d: added %d bufs, prod_idx=%u\n",
	    __func__, queue, n, prod_index);

	return (0);
}
