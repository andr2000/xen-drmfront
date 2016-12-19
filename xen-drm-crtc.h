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

#define XENDRM_CRTC_VREFRESH_HZ	60
/* timeout for page flip event reception */
#define XENDRM_CRTC_PFLIP_TO_MS	1000

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
	/* vblank and flip handling */
	atomic_t pg_flip_pending;
	atomic_t pg_flip_senders;
	struct drm_pending_vblank_event *pg_flip_event;
	wait_queue_head_t flip_wait;
};

int xendrm_du_crtc_create(struct xendrm_du_device *xendrm_du,
	struct xendrm_du_crtc *du_crtc, unsigned int index);
int xendrm_du_encoder_create(struct xendrm_du_device *xendrm_du,
	struct xendrm_du_crtc *du_crtc);
int xendrm_du_connector_create(struct xendrm_du_device *xendrm_du,
	struct xendrm_du_crtc *du_crtc, struct xendrm_cfg_connector *cfg);

void xendrm_du_crtc_on_page_flip_done(struct xendrm_du_crtc *du_crtc, uint64_t fb_cookie);
void xendrm_du_crtc_on_page_flip_to(struct xendrm_du_crtc *du_crtc);

#endif /* __XEN_DRM_CRTC_H_ */
