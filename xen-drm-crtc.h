/*
 *  Xen para-virtual DRM device
 *
 *  Based on drivers/gpu/drm/rcar-du
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

#ifndef __XEN_DRM_CRTC_H_
#define __XEN_DRM_CRTC_H_

#include <drm/drmP.h>
#include <drm/drm_crtc.h>

struct xendrm_du_group;

/**
 * struct rcar_du_crtc - the CRTC, representing a DU superposition processor
 * @crtc: base DRM CRTC
 * @clock: the CRTC functional clock
 * @extclock: external pixel dot clock (optional)
 * @mmio_offset: offset of the CRTC registers in the DU MMIO block
 * @index: CRTC software and hardware index
 * @started: whether the CRTC has been started and is running
 * @event: event to post when the pending page flip completes
 * @flip_wait: wait queue used to signal page flip completion
 * @outputs: bitmask of the outputs (enum rcar_du_output) driven by this CRTC
 * @group: CRTC group this CRTC belongs to
 */
struct xendrm_du_crtc {
	struct drm_crtc crtc;

	struct clk *clock;
	struct clk *extclock;
	unsigned int mmio_offset;
	unsigned int index;
	bool started;

	struct drm_pending_vblank_event *event;
	wait_queue_head_t flip_wait;

	unsigned int outputs;

	struct rcar_du_vsp *vsp;
};

int xendrm_du_crtc_create(unsigned int index);

#endif /* __XEN_DRM_CRTC_H_ */
