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

#include <linux/kernel.h>
#include <linux/list.h>

#include <xen/xen.h>
#include <asm/xen/hypercall.h>
#include <xen/interface/memory.h>
#include <xen/page.h>

#include "xen-drm-balloon.h"

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

static enum bp_state increase_reservation(unsigned long nr_pages)
{
	int rc;
	unsigned long i;
	struct page   *page;
	struct xen_memory_reservation reservation = {
		.address_bits = 0,
		.extent_order = EXTENT_ORDER,
		.domid        = DOMID_SELF
	};

	if (nr_pages > ARRAY_SIZE(frame_list))
		nr_pages = ARRAY_SIZE(frame_list);

	page = list_first_entry_or_null(&ballooned_pages, struct page, lru);
	for (i = 0; i < nr_pages; i++) {
		if (!page) {
			nr_pages = i;
			break;
		}

		/* XENMEM_populate_physmap requires a PFN based on Xen
		 * granularity.
		 */
		frame_list[i] = page_to_xen_pfn(page);
		page = balloon_next_page(page);
	}

	set_xen_guest_handle(reservation.extent_start, frame_list);
	reservation.nr_extents = nr_pages;
	rc = HYPERVISOR_memory_op(XENMEM_populate_physmap, &reservation);
	if (rc <= 0)
		return BP_EAGAIN;

	for (i = 0; i < rc; i++) {
		page = balloon_retrieve(false);
		BUG_ON(page == NULL);

#ifdef CONFIG_XEN_HAVE_PVMMU
		/*
		 * We don't support PV MMU when Linux and Xen is using
		 * different page granularity.
		 */
		BUILD_BUG_ON(XEN_PAGE_SIZE != PAGE_SIZE);

		if (!xen_feature(XENFEAT_auto_translated_physmap)) {
			unsigned long pfn = page_to_pfn(page);

			set_phys_to_machine(pfn, frame_list[i]);

			/* Link back into the page tables if not highmem. */
			if (!PageHighMem(page)) {
				int ret;
				ret = HYPERVISOR_update_va_mapping(
						(unsigned long)__va(pfn << PAGE_SHIFT),
						mfn_pte(frame_list[i], PAGE_KERNEL),
						0);
				BUG_ON(ret);
			}
		}
#endif

		/* Relinquish the page back to the allocator. */
		__free_reserved_page(page);
	}

	balloon_stats.current_pages += rc;

	return BP_DONE;
}

static enum bp_state decrease_reservation(unsigned long nr_pages, gfp_t gfp)
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
