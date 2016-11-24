/*
 *  Xen para-virtual DRM device
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

#include <linux/wait.h>

struct xendrm_du_device;
struct xendrm_cfg_connector;

struct xendrm_du_connector {
	struct drm_connector base;
	int width, height;
};

struct xendrm_du_crtc {
	int index;
	struct xendrm_du_device *xendrm_du;
	struct drm_plane primary;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct xendrm_du_connector connector;
	struct {
		struct drm_property *alpha;
	} props;
	bool enabled;
	/* vblank and flip handling */
	struct drm_pending_vblank_event *pg_flip_event;
	bool pg_flip_flush_queued;
	bool pg_flip_be_ntfy_fired;
	wait_queue_head_t flip_wait;
	struct timer_list timer_vblank;
	spinlock_t timer_lock;
	unsigned long timer_period;
	int timer_pf_event_to_cnt;
	int timer_pf_event_to;
};

int xendrm_du_crtc_create(struct xendrm_du_device *xendrm_du,
	struct xendrm_du_crtc *du_crtc, unsigned int index);
int xendrm_du_encoder_create(struct xendrm_du_device *xendrm_du,
	struct xendrm_du_crtc *du_crtc);
int xendrm_du_connector_create(struct xendrm_du_device *xendrm_du,
	struct xendrm_du_crtc *du_crtc, struct xendrm_cfg_connector *cfg);

void xendrm_du_crtc_on_page_flip(struct xendrm_du_crtc *du_crtc, uint64_t fb_cookie);
void xendrm_du_crtc_enable_vblank(struct xendrm_du_crtc *du_crtc, bool enable);

#endif /* __XEN_DRM_CRTC_H_ */
