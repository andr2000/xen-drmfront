/*
 *  Xen para-virtual DRM device
 *
 *   Based on: drivers/xen/balloon.c
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 * Copyright (C) 2016 EPAM Systems Inc.
 */

#include <linux/list.h>

#include <xen/xen.h>
#include <asm/xen/hypercall.h>
#include <xen/page.h>

#include "xen-drm-balloon.h"

/*
 * Use one extent per PAGE_SIZE to avoid to break down the page into
 * multiple frame.
 */
#define EXTENT_ORDER (fls(XEN_PFN_PER_PAGE) - 1)

/*
 * balloon_process() state:
 *
 * BP_DONE: done or nothing to do,
 * BP_WAIT: wait to be rescheduled,
 * BP_EAGAIN: error, go to sleep,
 * BP_ECANCELED: error, balloon operation canceled.
 */

enum bp_state {
	BP_DONE,
	BP_WAIT,
	BP_EAGAIN,
	BP_ECANCELED
};

static enum bp_state increase_reservation(struct xen_drm_balloon *balloon,
	unsigned long nr_pages, struct page **pages)
{
	int rc;
	unsigned long i, j, num_pages_in_batch;
	struct xen_memory_reservation reservation = {
		.address_bits = 0,
		.extent_order = EXTENT_ORDER,
		.domid        = DOMID_SELF
	};

	j = 0;
	while (nr_pages) {
		/* will rest of the pages fit in this batch? */
		num_pages_in_batch = nr_pages;
		if (num_pages_in_batch > ARRAY_SIZE(balloon->frame_list))
			num_pages_in_batch = ARRAY_SIZE(balloon->frame_list);
		for (i = 0; i < num_pages_in_batch; i++) {
			/* XENMEM_populate_physmap requires a PFN based on Xen
			 * granularity.
			 */
			balloon->frame_list[i] = page_to_xen_pfn(pages[j++]);
		}
		nr_pages -= num_pages_in_batch;
		set_xen_guest_handle(reservation.extent_start,
			balloon->frame_list);
		reservation.nr_extents = num_pages_in_batch;
		/* rc will hold number of pages processed */
		rc = HYPERVISOR_memory_op(XENMEM_populate_physmap,
			&reservation);
		if (rc <= 0)
			return BP_EAGAIN;

		/* Do not relinquish pages back to the allocator
		 * as after un-mapping we do need some memory
		 */
	}
	return BP_DONE;
}

static enum bp_state decrease_reservation(struct xen_drm_balloon *balloon,
	unsigned long nr_pages, struct page **pages)
{
	enum bp_state state = BP_DONE;
	unsigned long i;
	struct page *page, *tmp;
	int ret;
	struct xen_memory_reservation reservation = {
		.address_bits = 0,
		.extent_order = EXTENT_ORDER,
		.domid        = DOMID_SELF
	};
	LIST_HEAD(pages);

	if (nr_pages > ARRAY_SIZE(frame_list))
		nr_pages = ARRAY_SIZE(frame_list);

	for (i = 0; i < nr_pages; i++) {
		page = alloc_page(gfp);
		if (page == NULL) {
			nr_pages = i;
			state = BP_EAGAIN;
			break;
		}
		scrub_page(page);
		list_add(&page->lru, &pages);
	}

	/*
	 * Ensure that ballooned highmem pages don't have kmaps.
	 *
	 * Do this before changing the p2m as kmap_flush_unused()
	 * reads PTEs to obtain pages (and hence needs the original
	 * p2m entry).
	 */
	kmap_flush_unused();

	/*
	 * Setup the frame, update direct mapping, invalidate P2M,
	 * and add to balloon.
	 */
	i = 0;
	list_for_each_entry_safe(page, tmp, &pages, lru) {
		/* XENMEM_decrease_reservation requires a GFN */
		frame_list[i++] = xen_page_to_gfn(page);

#ifdef CONFIG_XEN_HAVE_PVMMU
		/*
		 * We don't support PV MMU when Linux and Xen is using
		 * different page granularity.
		 */
		BUILD_BUG_ON(XEN_PAGE_SIZE != PAGE_SIZE);

		if (!xen_feature(XENFEAT_auto_translated_physmap)) {
			unsigned long pfn = page_to_pfn(page);

			if (!PageHighMem(page)) {
				ret = HYPERVISOR_update_va_mapping(
						(unsigned long)__va(pfn << PAGE_SHIFT),
						__pte_ma(0), 0);
				BUG_ON(ret);
			}
			__set_phys_to_machine(pfn, INVALID_P2M_ENTRY);
		}
#endif
		list_del(&page->lru);

		balloon_append(page);
	}

	flush_tlb_all();

	set_xen_guest_handle(reservation.extent_start, frame_list);
	reservation.nr_extents   = nr_pages;
	ret = HYPERVISOR_memory_op(XENMEM_decrease_reservation, &reservation);
	BUG_ON(ret != nr_pages);

	balloon_stats.current_pages -= nr_pages;

	return state;
}
