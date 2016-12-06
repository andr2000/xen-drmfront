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
#include "xen-drm-logs.h"

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
	int nr_pages, struct page **pages)
{
	int rc;
	int i, j, num_pages_in_batch;
	struct xen_memory_reservation reservation = {
		.address_bits = 0,
		.extent_order = EXTENT_ORDER,
		.domid        = DOMID_SELF
	};

	j = 0;
	while (nr_pages) {
again:
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
		set_xen_guest_handle(reservation.extent_start,
			balloon->frame_list);
		reservation.nr_extents = num_pages_in_batch;
		/* rc will hold number of pages processed */
		rc = HYPERVISOR_memory_op(XENMEM_populate_physmap,
			&reservation);
		if (rc <= 0) {
			LOG0("Failed to populate physmap: requested %d done %d, retrying",
				num_pages_in_batch, rc);
			j -= num_pages_in_batch;
			goto again;
		}
		nr_pages -= num_pages_in_batch;
		/* Do not relinquish pages back to the allocator
		 * as after un-mapping we do need some memory
		 */
	}
	return BP_DONE;
}

static enum bp_state decrease_reservation(struct xen_drm_balloon *balloon,
	int nr_pages, struct page **pages)
{
	int i, j, num_pages_in_batch;
	int rc;
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
			balloon->frame_list[i] = xen_page_to_gfn(pages[j++]);
		}
		nr_pages -= num_pages_in_batch;
		set_xen_guest_handle(reservation.extent_start,
			balloon->frame_list);
		reservation.nr_extents = num_pages_in_batch;
		/* rc will hold number of pages processed */
		rc = HYPERVISOR_memory_op(XENMEM_decrease_reservation,
			&reservation);
		BUG_ON(rc != nr_pages);
	}
	return BP_DONE;
}
