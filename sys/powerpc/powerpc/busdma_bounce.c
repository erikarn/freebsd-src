/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 1997, 1998 Justin T. Gibbs.
 * Copyright (c) 2026 Raptor Engineering, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification, immediately at the beginning of the file.
 * 2. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * From amd64/busdma_machdep.c, r204214
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/domainset.h>
#include <sys/malloc.h>
#include <sys/bus.h>
#include <sys/interrupt.h>
#include <sys/kernel.h>
#include <sys/ktr.h>
#include <sys/lock.h>
#include <sys/proc.h>
#include <sys/memdesc.h>
#include <sys/mutex.h>
#include <sys/sysctl.h>
#include <sys/uio.h>

#include <vm/vm.h>
#include <vm/vm_extern.h>
#include <vm/vm_kern.h>
#include <vm/vm_page.h>
#include <vm/vm_map.h>

#include <machine/atomic.h>
#include <machine/bus.h>
#include <machine/cpufunc.h>
#include <machine/md_var.h>
#include <machine/bus_dma_impl.h>

#include "iommu_if.h"

#define MAX_BPAGES MIN(8192, physmem/40)

enum {
	BF_COULD_BOUNCE         = 0x01,
	BF_MIN_ALLOC_COMP       = 0x02,
	BF_KMEM_ALLOC		= 0x04,
	BF_COHERENT		= 0x10,
};

struct bounce_page;
struct bounce_zone;

struct bus_dma_tag {
	struct bus_dma_tag_common common;
	int		  map_count;
	int		  bounce_flags;
	struct bounce_zone *bounce_zone;
	device_t	  iommu;
	void		 *iommu_cookie;
};

static SYSCTL_NODE(_hw, OID_AUTO, busdma, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "Busdma parameters");

struct sync_list {
	vm_offset_t	vaddr;		/* kva of client data */
	bus_addr_t	paddr;		/* physical address */
	vm_page_t	pages;		/* starting page of client data */
	bus_size_t	datacount;	/* client data count */
};

struct bus_dmamap {
	STAILQ_HEAD(, bounce_page) bpages;
	int		       pagesneeded;
	int		       pagesreserved;
	bus_dma_tag_t	       dmat;
	struct memdesc	       mem;

	/* temporary glue for existing pSeries IOMMU setup */
	bus_dma_segment_t     *segments;
	int		       nsegs;

	bus_dmamap_callback_t *callback;
	void		      *callback_arg;
	__sbintime_t	       queued_time;
	STAILQ_ENTRY(bus_dmamap) links;
	u_int			flags;
#define	DMAMAP_COHERENT		(1 << 0)
#define	DMAMAP_COULD_BOUNCE	(1 << 1)
#define	DMAMAP_FROM_DMAMEM	(1 << 2)
#define	DMAMAP_MBUF		(1 << 3)
	int			sync_count;
	struct sync_list	slist[];
};

static bool _bus_dmamap_pagesneeded(bus_dma_tag_t dmat, bus_dmamap_t map,
    vm_paddr_t buf, bus_size_t buflen, int *pagesneeded);
static void _bus_dmamap_count_pages(bus_dma_tag_t dmat, bus_dmamap_t map,
    pmap_t pmap, void *buf, bus_size_t buflen, int flags);
static void _bus_dmamap_count_phys(bus_dma_tag_t dmat, bus_dmamap_t map,
    vm_paddr_t buf, bus_size_t buflen, int flags);

static MALLOC_DEFINE(M_BUSDMA, "busdma", "busdma metadata");

#define	dmat_alignment(dmat)	((dmat)->common.alignment)
#define	dmat_bounce_flags(dmat)	((dmat)->bounce_flags)
#define	dmat_boundary(dmat)	((dmat)->common.boundary)
#define dmat_domain(dmat)       ((dmat)->common.domain)
#define	dmat_flags(dmat)	((dmat)->common.flags)
#define	dmat_highaddr(dmat)	((dmat)->common.highaddr)
#define	dmat_lowaddr(dmat)	((dmat)->common.lowaddr)
#define	dmat_lockfunc(dmat)	((dmat)->common.lockfunc)
#define	dmat_lockfuncarg(dmat)	((dmat)->common.lockfuncarg)
#define	dmat_maxsegsz(dmat)	((dmat)->common.maxsegsz)
#define	dmat_nsegments(dmat)	((dmat)->common.nsegments)

#include "../../kern/subr_busdma_bounce.c"

static int
bounce_bus_dma_zone_setup(bus_dma_tag_t dmat)
{
	struct bounce_zone *bz;
	bus_size_t maxsize;
	int error;

	/*
	 * Round size up to a full page, and add one more page because
	 * there can always be one more boundary crossing than the
	 * number of pages in a transfer.
	 */
	maxsize = roundup2(dmat->common.maxsize, PAGE_SIZE) + PAGE_SIZE;

	/* Must bounce */
	if ((error = alloc_bounce_zone(dmat)) != 0)
		return (error);
	bz = dmat->bounce_zone;

	if (ptoa(bz->total_bpages) < maxsize) {
		int pages;

		pages = atop(maxsize) + 1 - bz->total_bpages;

		/* Add pages to our bounce pool */
		if (alloc_bounce_pages(dmat, pages) < pages)
			return (ENOMEM);
	}
	/* Performed initial allocation */
	dmat->bounce_flags |= BF_MIN_ALLOC_COMP;

	return (error);
}

/*
 * Return true if the DMA should bounce because the start or end does not fall
 * on a cacheline boundary (which would require a partial cacheline flush).
 * COHERENT memory doesn't trigger cacheline flushes.  Memory allocated by
 * bus_dmamem_alloc() is always aligned to cacheline boundaries, and there's a
 * strict rule that such memory cannot be accessed by the CPU while DMA is in
 * progress (or by multiple DMA engines at once), so that it's always safe to do
 * full cacheline flushes even if that affects memory outside the range of a
 * given DMA operation that doesn't involve the full allocated buffer.  If we're
 * mapping an mbuf, that follows the same rules as a buffer we allocated.
 */
static bool
cacheline_bounce(bus_dma_tag_t dmat, bus_dmamap_t map, bus_addr_t paddr,
    bus_size_t size)
{

#define	DMAMAP_CACHELINE_FLAGS						\
    (DMAMAP_FROM_DMAMEM | DMAMAP_COHERENT | DMAMAP_MBUF)
	if ((dmat->bounce_flags & BF_COHERENT) != 0)
		return (false);
	if (map != NULL && (map->flags & DMAMAP_CACHELINE_FLAGS) != 0)
		return (false);
	return (((paddr | size) & (cacheline_size - 1)) != 0);
#undef DMAMAP_CACHELINE_FLAGS
}

/*
 * Return true if the given address does not fall on the alignment boundary.
 */
static bool
alignment_bounce(bus_dma_tag_t dmat, bus_addr_t addr)
{

	return (!vm_addr_align_ok(addr, dmat->common.alignment));
}

static bool
might_bounce(bus_dma_tag_t dmat, bus_dmamap_t map, bus_addr_t paddr,
    bus_size_t size)
{

	/* Memory allocated by bounce_bus_dmamem_alloc won't bounce */
	if (map && (map->flags & DMAMAP_FROM_DMAMEM) != 0)
		return (false);

	if ((dmat->bounce_flags & BF_COULD_BOUNCE) != 0)
		return (true);

	if (cacheline_bounce(dmat, map, paddr, size))
		return (true);

	if (alignment_bounce(dmat, paddr))
		return (true);

	return (false);
}

/*
 * Returns true if the address falls within the tag's exclusion window, or
 * fails to meet its alignment requirements.
 */
static bool
must_bounce(bus_dma_tag_t dmat, bus_dmamap_t map, bus_addr_t paddr,
    bus_size_t size)
{
	if (cacheline_bounce(dmat, map, paddr, size))
		return (true);

	if (dmat->iommu == NULL && paddr > dmat->common.lowaddr &&
	    paddr <= dmat->common.highaddr)
		return (true);

	if (!vm_addr_align_ok(paddr, dmat->common.alignment))
		return (true);

	if ((dmat->bounce_flags & BF_COULD_BOUNCE) != 0 &&
	    addr_needs_bounce(dmat, paddr))
		return (true);

	return (false);
}

/*
 * Allocate a device specific dma_tag.
 */
static int
bounce_bus_dma_tag_create(bus_dma_tag_t parent, bus_size_t alignment,
		   bus_addr_t boundary, bus_addr_t lowaddr,
		   bus_addr_t highaddr, bus_size_t maxsize, int nsegments,
		   bus_size_t maxsegsz, int flags, bus_dma_lock_t *lockfunc,
		   void *lockfuncarg, bus_dma_tag_t *dmat)
{
	bus_dma_tag_t newtag;
	int error = 0;

	/* Basic sanity checking */
	if (boundary != 0 && boundary < maxsegsz)
		maxsegsz = boundary;

	if (maxsegsz == 0) {
		return (EINVAL);
	}

	/* Return a NULL tag on failure */
	*dmat = NULL;
	error = common_bus_dma_tag_create(parent != NULL ? &parent->common :
	    NULL, alignment, boundary, lowaddr, highaddr, maxsize, nsegments,
	    maxsegsz, flags, lockfunc, lockfuncarg,
	    sizeof (struct bus_dma_tag), (void **)&newtag);
	if (error != 0)
		return (error);

	newtag->common.impl = &bus_dma_bounce_impl;
	newtag->map_count = 0;

	if ((flags & BUS_DMA_COHERENT) != 0)
		newtag->bounce_flags |= BF_COHERENT;

	if (parent != NULL) {
		newtag->iommu = parent->iommu;
		newtag->iommu_cookie = parent->iommu_cookie;

		if ((parent->bounce_flags & BF_COULD_BOUNCE) != 0)
			newtag->bounce_flags |= BF_COULD_BOUNCE;

		/* Copy some flags from the parent */
		newtag->bounce_flags |= parent->bounce_flags & BF_COHERENT;
	}

	if ((newtag->common.lowaddr < ptoa((vm_paddr_t)Maxmem) ||
	    (newtag->common.alignment > 1)) && newtag->iommu == NULL)
		newtag->bounce_flags |= BF_COULD_BOUNCE;

	if ((flags & BUS_DMA_ALLOCNOW) != 0)
		error = bounce_bus_dma_zone_setup(newtag);
	else
		error = 0;

	if (error != 0) {
		free(newtag, M_DEVBUF);
	} else {
		*dmat = newtag;
	}
	CTR4(KTR_BUSDMA, "%s returned tag %p tag flags 0x%x error %d\n",
	    __func__, newtag, (newtag != NULL ? newtag->common.flags : 0), error);
	return (error);
}

static int
bounce_bus_dma_tag_destroy(bus_dma_tag_t dmat)
{
	int error = 0;

	if (dmat != NULL) {
		if (dmat->map_count != 0) {
			error = EBUSY;
			goto out;
		}

		free(dmat, M_DEVBUF);
	}
out:
	CTR3(KTR_BUSDMA, "%s tag %p error %d\n", __func__, dmat, error);
	return (error);
}

/*
 * Update the domain for the tag.  We may need to reallocate the zone and
 * bounce pages.
 */
static int
bounce_bus_dma_tag_set_domain(bus_dma_tag_t dmat)
{

	KASSERT(dmat->map_count == 0,
	    ("bounce_bus_dma_tag_set_domain:  Domain set after use.\n"));
	if ((dmat->bounce_flags & BF_COULD_BOUNCE) == 0 ||
	    dmat->bounce_zone == NULL)
		return (0);
	dmat->bounce_flags &= ~BF_MIN_ALLOC_COMP;
	return (bounce_bus_dma_zone_setup(dmat));
}

static bool
bounce_bus_dma_id_mapped(bus_dma_tag_t dmat, vm_paddr_t buf, bus_size_t buflen)
{

	if (!might_bounce(dmat, NULL, buf, buflen))
		return (true);
	return (!_bus_dmamap_pagesneeded(dmat, NULL, buf, buflen, NULL));
}

static bus_dmamap_t
alloc_dmamap(bus_dma_tag_t dmat, int flags)
{
	u_long mapsize;
	bus_dmamap_t map;

	mapsize = sizeof(*map);
	mapsize += sizeof(struct sync_list) * dmat->common.nsegments;
	map = malloc(mapsize, M_DEVBUF, flags | M_ZERO);
	if (map == NULL)
		return (NULL);

	/* Initialize the new map */
	STAILQ_INIT(&map->bpages);

	return (map);
}

/*
 * Allocate a handle for mapping from kva/uva/physical
 * address space into bus device space.
 */
static int
bounce_bus_dmamap_create(bus_dma_tag_t dmat, int flags, bus_dmamap_t *mapp)
{
	int error;

	error = 0;

	*mapp = alloc_dmamap(dmat, M_NOWAIT);
	if (*mapp == NULL) {
		CTR3(KTR_BUSDMA, "%s: tag %p error %d\n",
		    __func__, dmat, ENOMEM);
		return (ENOMEM);
	}

	/*
	 * Bouncing might be required if the driver asks for an active
	 * exclusion region, a data alignment that is stricter than 1, and/or
	 * an active address boundary.
	 */
	if (dmat->bounce_flags & BF_COULD_BOUNCE) {
		/* Must bounce */
		struct bounce_zone *bz;
		int maxpages;

		if (dmat->bounce_zone == NULL) {
			if ((error = alloc_bounce_zone(dmat)) != 0)
				return (error);
		}
		bz = dmat->bounce_zone;

		(*mapp)->flags = DMAMAP_COULD_BOUNCE;

		/*
		 * Attempt to add pages to our pool on a per-instance
		 * basis up to a sane limit.
		 */
		if (dmat->common.alignment > 1)
			maxpages = MAX_BPAGES;
		else
			maxpages = MIN(MAX_BPAGES, Maxmem -atop(dmat->common.lowaddr));
		if ((dmat->bounce_flags & BF_MIN_ALLOC_COMP) == 0
		 || (bz->map_count > 0 && bz->total_bpages < maxpages)) {
			int pages;

			pages = MAX(atop(dmat->common.maxsize), 1);
			pages = MIN(maxpages - bz->total_bpages, pages);
			pages = MAX(pages, 1);
			if (alloc_bounce_pages(dmat, pages) < pages)
				error = ENOMEM;

			if ((dmat->bounce_flags & BF_MIN_ALLOC_COMP) == 0) {
				if (error == 0)
					dmat->bounce_flags |= BF_MIN_ALLOC_COMP;
			} else {
				error = 0;
			}
		}
		bz->map_count++;
	}

	/* temporary glue for existing pSeries IOMMU setup */
	(*mapp)->nsegs = 0;
	(*mapp)->segments = (bus_dma_segment_t *)malloc(
	    sizeof(bus_dma_segment_t) * dmat->common.nsegments, M_DEVBUF,
	    M_NOWAIT);
	if ((*mapp)->segments == NULL) {
		CTR3(KTR_BUSDMA, "%s: tag %p error %d",
		    __func__, dmat, ENOMEM);
		return (ENOMEM);
	}

	if (error == 0) {
		dmat->map_count++;
		if ((dmat->bounce_flags & BF_COHERENT) != 0)
			(*mapp)->flags |= DMAMAP_COHERENT;
	} else {
		free(*mapp, M_DEVBUF);
	}
	CTR4(KTR_BUSDMA, "%s: tag %p tag flags 0x%x error %d\n",
	    __func__, dmat, dmat->common.flags, error);
	return (error);
}

/*
 * Destroy a handle for mapping from kva/uva/physical
 * address space into bus device space.
 */
static int
bounce_bus_dmamap_destroy(bus_dma_tag_t dmat, bus_dmamap_t map)
{
	/* Check we are destroying the correct map type */
	if ((map->flags & DMAMAP_FROM_DMAMEM) != 0)
		panic("bounce_bus_dmamap_destroy: Invalid map freed\n");

	if (STAILQ_FIRST(&map->bpages) != NULL || map->sync_count != 0) {
		CTR3(KTR_BUSDMA, "%s: tag %p error %d\n", __func__, dmat, EBUSY);
		return (EBUSY);
	}
	if (dmat->bounce_zone) {
		KASSERT((map->flags & DMAMAP_COULD_BOUNCE) != 0,
		    ("%s: Bounce zone when cannot bounce", __func__));
		dmat->bounce_zone->map_count--;
	}
	free(map->segments, M_DEVBUF);
	free(map, M_DEVBUF);
	dmat->map_count--;
	CTR2(KTR_BUSDMA, "%s: tag %p error 0\n", __func__, dmat);
	return (0);
}

/*
 * Allocate a piece of memory that can be efficiently mapped into
 * bus device space based on the constraints lited in the dma tag.
 * A dmamap to for use with dmamap_load is also allocated.
 */
static int
bounce_bus_dmamem_alloc(bus_dma_tag_t dmat, void** vaddr, int flags,
		 bus_dmamap_t *mapp)
{
	vm_memattr_t attr;
	int mflags;

	if (flags & BUS_DMA_NOWAIT)
		mflags = M_NOWAIT;
	else
		mflags = M_WAITOK;

	bounce_bus_dmamap_create(dmat, mflags, mapp);
	if (*mapp == NULL) {
		CTR4(KTR_BUSDMA, "%s: tag %p tag flags 0x%x error %d\n",
		    __func__, dmat, dmat->common.flags, ENOMEM);
		return (ENOMEM);
	}

	if (flags & BUS_DMA_ZERO)
		mflags |= M_ZERO;
	if (flags & BUS_DMA_NOCACHE)
		attr = VM_MEMATTR_UNCACHEABLE;
#if 0
	else if ((flags & BUS_DMA_COHERENT) != 0 &&
	    (dmat->bounce_flags & BF_COHERENT) == 0)
		/*
		 * If we have a non-coherent tag, and are trying to allocate
		 * a coherent block of memory it needs to be uncached.
		 */
		attr = VM_MEMATTR_UNCACHEABLE;
#endif
	else
		attr = VM_MEMATTR_DEFAULT;

	/*
	 * Mark the map as coherent if we used uncacheable memory or the
	 * tag was already marked as coherent.
	 */
	if (attr == VM_MEMATTR_UNCACHEABLE ||
	    (dmat->bounce_flags & BF_COHERENT) != 0)
		(*mapp)->flags |= DMAMAP_COHERENT;

	(*mapp)->flags = DMAMAP_FROM_DMAMEM;

	/*
	 * XXX:
	 * (dmat->common.alignment <= dmat->common.maxsize) is just a quick hack; the exact
	 * alignment guarantees of malloc need to be nailed down, and the
	 * code below should be rewritten to take that into account.
	 *
	 * In the meantime, we'll warn the user if malloc gets it wrong.
	 */
	if ((dmat->common.maxsize <= PAGE_SIZE) &&
	   (dmat->common.alignment <= dmat->common.maxsize) &&
	    dmat->common.lowaddr >= ptoa((vm_paddr_t)Maxmem) &&
	    attr == VM_MEMATTR_DEFAULT) {
		*vaddr = malloc(dmat->common.maxsize, M_DEVBUF, mflags);
	} else {
		/*
		 * XXX Use Contigmalloc until it is merged into this facility
		 *     and handles multi-seg allocations.  Nobody is doing
		 *     multi-seg allocations yet though.
		 * XXX Certain AGP hardware does.
		 */
		*vaddr = kmem_alloc_contig(dmat->common.maxsize, mflags, 0ul,
		    dmat->common.lowaddr, dmat->common.alignment ? dmat->common.alignment : 1ul,
		    dmat->common.boundary, attr);
		dmat->bounce_flags |= BF_KMEM_ALLOC;
	}
	if (*vaddr == NULL) {
		CTR4(KTR_BUSDMA, "%s: tag %p tag flags 0x%x error %d\n",
		    __func__, dmat, dmat->common.flags, ENOMEM);
		return (ENOMEM);
	} else if (!vm_addr_align_ok(vtophys(*vaddr), dmat->common.alignment)) {
		printf("bus_dmamem_alloc failed to align memory properly.\n");
	}
	CTR4(KTR_BUSDMA, "%s: tag %p tag flags 0x%x error %d\n",
	    __func__, dmat, dmat->common.flags, 0);
	return (0);
}

/*
 * Free a piece of memory and it's allociated dmamap, that was allocated
 * via bus_dmamem_alloc.
 */
static void
bounce_bus_dmamem_free(bus_dma_tag_t dmat, void *vaddr, bus_dmamap_t map)
{
	/*
	 * Check the map came from bounce_bus_dmamem_alloc, so the map
	 * should be NULL and the BF_KMEM_ALLOC flag cleared if malloc()
	 * was used and set if kmem_alloc_contig() was used.
	 */
	if ((map->flags & DMAMAP_FROM_DMAMEM) == 0)
		panic("bus_dmamem_free: Invalid map freed\n");
	if ((dmat->bounce_flags & BF_KMEM_ALLOC) == 0)
		free(vaddr, M_DEVBUF);
	else
		kmem_free(vaddr, dmat->common.maxsize);
	free(map, M_DEVBUF);
	dmat->map_count--;
	CTR3(KTR_BUSDMA, "%s: tag %p flags 0x%x", __func__, dmat,
	    dmat->bounce_flags);
}

static bool
_bus_dmamap_pagesneeded(bus_dma_tag_t dmat, bus_dmamap_t map, vm_paddr_t buf,
    bus_size_t buflen, int *pagesneeded)
{
	bus_addr_t curaddr;
	bus_size_t sgsize;
	int count;

	/*
	 * Count the number of bounce pages needed in order to
	 * complete this transfer
	 */
	count = 0;
	curaddr = buf;
	while (buflen != 0) {
		sgsize = buflen;
		if (must_bounce(dmat, map, curaddr, sgsize)) {
			sgsize = MIN(sgsize,
			    PAGE_SIZE - (curaddr & PAGE_MASK));
			if (pagesneeded == NULL)
				return (true);
			count++;
		}
		curaddr += sgsize;
		buflen -= sgsize;
	}

	if (pagesneeded != NULL)
		*pagesneeded = count;
	return (count != 0);
}

static void
_bus_dmamap_count_phys(bus_dma_tag_t dmat, bus_dmamap_t map, vm_paddr_t buf,
    bus_size_t buflen, int flags)
{

	if ((map->flags & DMAMAP_COULD_BOUNCE) != 0 && map->pagesneeded == 0) {
		CTR4(KTR_BUSDMA, "lowaddr= 0x%lx Maxmem= %ld, boundary= %ld, "
		    "alignment= %ld", dmat->common.lowaddr, ptoa((vm_paddr_t)Maxmem),
		    dmat->common.boundary, dmat->common.alignment);
		CTR2(KTR_BUSDMA, "map= %p, pagesneeded= %d", map, map->pagesneeded);
		_bus_dmamap_pagesneeded(dmat, map, buf, buflen,
		    &map->pagesneeded);
		_bus_dmamap_pagesneeded(dmat, map, buf, buflen,
		    &map->pagesneeded);
		CTR1(KTR_BUSDMA, "pagesneeded= %d\n", map->pagesneeded);
	}
}

static void
_bus_dmamap_count_pages(bus_dma_tag_t dmat, bus_dmamap_t map, pmap_t pmap,
    void *buf, bus_size_t buflen, int flags)
{
        vm_offset_t vaddr;
        vm_offset_t vendaddr;
        bus_addr_t paddr;

	if ((map->flags & DMAMAP_COULD_BOUNCE) != 0 && map->pagesneeded == 0) {
		CTR4(KTR_BUSDMA, "lowaddr= 0x%lx Maxmem= %ld, boundary= %ld, "
		    "alignment= %ld", dmat->common.lowaddr, ptoa((vm_paddr_t)Maxmem),
		    dmat->common.boundary, dmat->common.alignment);
		CTR2(KTR_BUSDMA, "map= %p, pagesneeded= %d", map, map->pagesneeded);
		/*
		 * Count the number of bounce pages
		 * needed in order to complete this transfer
		 */
		vaddr = (vm_offset_t)buf;
		vendaddr = (vm_offset_t)buf + buflen;

		while (vaddr < vendaddr) {
			bus_size_t sg_len;

			sg_len = MIN(vendaddr - vaddr,
			    PAGE_SIZE - ((vm_offset_t)vaddr & PAGE_MASK));
			if (pmap == kernel_pmap)
				paddr = pmap_kextract(vaddr);
			else
				paddr = pmap_extract(pmap, vaddr);
			if (must_bounce(dmat, map, paddr, sg_len)) {
				sg_len = roundup2(sg_len, dmat->common.alignment);
				map->pagesneeded++;
			}
			vaddr += sg_len;
		}
		CTR1(KTR_BUSDMA, "pagesneeded= %d\n", map->pagesneeded);
	}
}

/*
 * Utility function to load a physical buffer.  segp contains
 * the starting segment on entrace, and the ending segment on exit.
 */
static int
bounce_bus_dmamap_load_phys(bus_dma_tag_t dmat,
		      bus_dmamap_t map,
		      vm_paddr_t buf, bus_size_t buflen,
		      int flags,
		      bus_dma_segment_t *segs,
		      int *segp)
{
	struct sync_list *sl;
	bus_size_t sgsize;
	bus_addr_t curaddr, sl_end;
	int error;

	if (segs == NULL)
		segs = map->segments;

	if ((dmat->bounce_flags & BF_COULD_BOUNCE) != 0) {
		_bus_dmamap_count_phys(dmat, map, buf, buflen, flags);
		if (map->pagesneeded != 0) {
			error = _bus_dmamap_reserve_pages(dmat, map, flags);
			if (error)
				return (error);
		}
	}

	sl = map->slist + map->sync_count - 1;
	sl_end = 0;

	while (buflen > 0) {
		curaddr = buf;
		sgsize = buflen;
		if (map->pagesneeded != 0 &&
		    must_bounce(dmat, map, curaddr, sgsize)) {
			sgsize = MIN(sgsize, PAGE_SIZE - (curaddr & PAGE_MASK));
			curaddr = add_bounce_page(dmat, map, 0, curaddr,
			    sgsize);
		} else if ((map->flags & DMAMAP_COHERENT) == 0) {
			if (map->sync_count > 0)
				sl_end = sl->paddr + sl->datacount;

			if (map->sync_count == 0 || curaddr != sl_end) {
				if (++map->sync_count > dmat->common.nsegments)
					break;
				sl++;
				sl->vaddr = 0;
				sl->paddr = curaddr;
				sl->datacount = sgsize;
				sl->pages = PHYS_TO_VM_PAGE(curaddr);
				KASSERT(sl->pages != NULL,
				    ("%s: page at PA:0x%08lx is not in "
				    "vm_page_array", __func__, curaddr));
			} else
				sl->datacount += sgsize;
		}
		if (!_bus_dmamap_addsegs(dmat, map, curaddr, sgsize, segs,
		    segp))
			break;
		buf += sgsize;
		buflen -= sgsize;
	}

	/*
	 * Did we fit?
	 */
	return (buflen != 0 ? EFBIG : 0); /* XXX better return value here? */
}

/*
 * Utility function to load a linear buffer.  segp contains
 * the starting segment on entrance, and the ending segment on exit.
 */
static int
bounce_bus_dmamap_load_buffer(bus_dma_tag_t dmat, bus_dmamap_t map, void *buf,
    bus_size_t buflen, pmap_t pmap, int flags, bus_dma_segment_t *segs,
    int *segp)
{
	struct sync_list *sl;
	bus_size_t sgsize;
	bus_addr_t curaddr, sl_pend;
	vm_offset_t kvaddr, vaddr, sl_vend;
	int error;

	if (segs == NULL)
		segs = map->segments;

	if (flags & BUS_DMA_LOAD_MBUF)
		map->flags |= DMAMAP_MBUF;

	if ((dmat->bounce_flags & BF_COULD_BOUNCE) != 0) {
		_bus_dmamap_count_pages(dmat, map, pmap, buf, buflen, flags);
		if (map->pagesneeded != 0) {
			error = _bus_dmamap_reserve_pages(dmat, map, flags);
			if (error)
				return (error);
		}
	}

	sl = map->slist + map->sync_count - 1;
	vaddr = (vm_offset_t)buf;
	sl_pend = 0;
	sl_vend = 0;

	while (buflen > 0) {
		/*
		 * Get the physical address for this segment.
		 */
		if (pmap == kernel_pmap) {
			curaddr = pmap_kextract(vaddr);
			kvaddr = vaddr;
		} else {
			curaddr = pmap_extract(pmap, vaddr);
			kvaddr = 0;
		}

		/*
		 * Compute the segment size, and adjust counts.
		 */
		sgsize = MIN(buflen, PAGE_SIZE - (curaddr & PAGE_MASK));
		if (map->pagesneeded != 0 &&
		    must_bounce(dmat, map, curaddr, sgsize)) {
			sgsize = roundup2(sgsize, dmat->common.alignment);
			sgsize = MIN(sgsize, buflen);
			curaddr = add_bounce_page(dmat, map, kvaddr, curaddr,
			    sgsize);
		} else if ((map->flags & DMAMAP_COHERENT) == 0) {
			if (map->sync_count > 0) {
				sl_pend = sl->paddr + sl->datacount;
				sl_vend = sl->vaddr + sl->datacount;
			}

			if (map->sync_count == 0 ||
			    (kvaddr != 0 && kvaddr != sl_vend) ||
			    (curaddr != sl_pend)) {
				if (++map->sync_count > dmat->common.nsegments)
					goto cleanup;
				sl++;
				sl->vaddr = kvaddr;
				sl->paddr = curaddr;
				if (kvaddr != 0) {
					sl->pages = NULL;
				} else {
					sl->pages = PHYS_TO_VM_PAGE(curaddr);
					KASSERT(sl->pages != NULL,
					    ("%s: page at PA:0x%08lx is not "
					    "in vm_page_array", __func__,
					    curaddr));
				}
				sl->datacount = sgsize;
			} else
				sl->datacount += sgsize;
		}

		if (!_bus_dmamap_addsegs(dmat, map, curaddr, sgsize, segs,
		    segp))
			break;
		vaddr += sgsize;
		buflen -= MIN(sgsize, buflen); /* avoid underflow */
	}

cleanup:
	/*
	 * Did we fit?
	 */
	return (buflen != 0 ? EFBIG : 0); /* XXX better return value here? */
}

static void
bounce_bus_dmamap_waitok(bus_dma_tag_t dmat, bus_dmamap_t map,
		    struct memdesc *mem, bus_dmamap_callback_t *callback,
		    void *callback_arg)
{

	if ((map->flags & DMAMAP_COULD_BOUNCE) == 0)
		return;

	map->mem = *mem;
	map->dmat = dmat;
	map->callback = callback;
	map->callback_arg = callback_arg;
}

static bus_dma_segment_t *
bounce_bus_dmamap_complete(bus_dma_tag_t dmat, bus_dmamap_t map,
		     bus_dma_segment_t *segs, int nsegs, int error)
{
	/* temporary glue for existing pSeries IOMMU setup */
	map->nsegs = nsegs;
	if (segs != NULL)
		memcpy(map->segments, segs, map->nsegs*sizeof(segs[0]));
	if (dmat->iommu != NULL)
		IOMMU_MAP(dmat->iommu, map->segments, &map->nsegs,
		    dmat->common.lowaddr, dmat->common.highaddr, dmat->common.alignment,
		    dmat->common.boundary, dmat->iommu_cookie);

	if (segs != NULL)
		memcpy(segs, map->segments, map->nsegs*sizeof(segs[0]));
	else
		segs = map->segments;
	return (segs);
}

/*
 * Release the mapping held by map.
 */
static void
bounce_bus_dmamap_unload(bus_dma_tag_t dmat, bus_dmamap_t map)
{
	/* temporary glue for existing pSeries IOMMU setup */
	if (dmat->iommu) {
		IOMMU_UNMAP(dmat->iommu, map->segments, map->nsegs,
		    dmat->iommu_cookie);
		map->nsegs = 0;
	}

	free_bounce_pages(dmat, map);
	map->sync_count = 0;
	map->flags &= ~DMAMAP_MBUF;
}

static void
bounce_bus_dmamap_sync(bus_dma_tag_t dmat, bus_dmamap_t map, bus_dmasync_op_t op)
{
	struct bounce_page *bpage;
	vm_offset_t datavaddr, tempvaddr;

	if ((bpage = STAILQ_FIRST(&map->bpages)) != NULL) {
		/*
		 * Handle data bouncing.  We might also
		 * want to add support for invalidating
		 * the caches on broken hardware
		 */
		CTR4(KTR_BUSDMA, "%s: tag %p tag flags 0x%x op 0x%x "
		    "performing bounce\n", __func__, dmat, dmat->common.flags, op);

		if (op & BUS_DMASYNC_PREWRITE) {
			while (bpage != NULL) {
				tempvaddr = 0;
				datavaddr = bpage->datavaddr;
				if (datavaddr == 0) {
					tempvaddr = pmap_quick_enter_page(
					    bpage->datapage);
					datavaddr = tempvaddr |
					    bpage->dataoffs;
				}

				bcopy((void *)datavaddr,
				    (void *)bpage->vaddr, bpage->datacount);

				if (tempvaddr != 0)
					pmap_quick_remove_page(tempvaddr);
				bpage = STAILQ_NEXT(bpage, links);
			}
			dmat->bounce_zone->total_bounced++;
		} else if ((op & BUS_DMASYNC_PREREAD) != 0) {
			/*
			 * Nothing to do here; ppc/ppc64 defaults to
			 * cache coherent.
			 */
		}

		if (op & BUS_DMASYNC_POSTREAD) {
			while (bpage != NULL) {
				tempvaddr = 0;
				datavaddr = bpage->datavaddr;
				if (datavaddr == 0) {
					tempvaddr = pmap_quick_enter_page(
					    bpage->datapage);
					datavaddr = tempvaddr |
					    bpage->dataoffs;
				}

				bcopy((void *)bpage->vaddr,
				    (void *)datavaddr, bpage->datacount);

				if (tempvaddr != 0)
					pmap_quick_remove_page(tempvaddr);
				bpage = STAILQ_NEXT(bpage, links);
			}
			dmat->bounce_zone->total_bounced++;
		}
	}

	/* Note: memory is coherent by default; no need to sync dcache */
	powerpc_sync();
}

int
bus_dma_tag_set_iommu(bus_dma_tag_t tag, device_t iommu, void *cookie)
{
	tag->iommu = iommu;
	tag->iommu_cookie = cookie;

	return (0);
}

struct bus_dma_impl bus_dma_bounce_impl = {
	.tag_create = bounce_bus_dma_tag_create,
	.tag_destroy = bounce_bus_dma_tag_destroy,
	.tag_set_domain = bounce_bus_dma_tag_set_domain,
	.id_mapped = bounce_bus_dma_id_mapped,
	.map_create = bounce_bus_dmamap_create,
	.map_destroy = bounce_bus_dmamap_destroy,
	.mem_alloc = bounce_bus_dmamem_alloc,
	.mem_free = bounce_bus_dmamem_free,
	.load_phys = bounce_bus_dmamap_load_phys,
	.load_buffer = bounce_bus_dmamap_load_buffer,
	.load_ma = bus_dmamap_load_ma_triv,
	.map_waitok = bounce_bus_dmamap_waitok,
	.map_complete = bounce_bus_dmamap_complete,
	.map_unload = bounce_bus_dmamap_unload,
	.map_sync = bounce_bus_dmamap_sync
};
