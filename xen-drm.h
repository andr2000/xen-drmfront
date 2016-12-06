/*
 *  Xen para-virtual DRM device
 *
 *  Based on
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

#ifndef __XEN_DRM_H
#define __XEN_DRM_H

#include <linux/platform_device.h>

#include "xen-drm-crtc.h"
#include "xen-drm-timer.h"

#define XENDRM_DU_MAX_CRTCS	4

struct xendispl_front_funcs;

struct xendrm_du_device {
	struct xdrv_info *xdrv_info;
	struct xendispl_front_funcs *front_funcs;
	struct drm_device *ddev;
	int num_crtcs;
	struct xendrm_plat_data *platdata;
	struct xendrm_du_crtc crtcs[XENDRM_DU_MAX_CRTCS];

	/* vblank and page flip handling */
	struct xendrm_du_timer vblank_timer;
	atomic_t pflip_to_cnt[XENDRM_DU_MAX_CRTCS];
	atomic_t pflip_to_cnt_armed[XENDRM_DU_MAX_CRTCS];
	atomic_t vblank_enabled[XENDRM_DU_MAX_CRTCS];
};

struct xendrm_cfg_connector {
	int width;
	int height;
	char *xenstore_path;
};

struct xendrm_plat_data {
	struct xdrv_info *xdrv_info;
	/* number of connectors in this configuration */
	int num_connectors;
	/* connector configurations */
	struct xendrm_cfg_connector connectors[XENDRM_DU_MAX_CRTCS];
};

int xendrm_probe(struct platform_device *pdev,
	struct xendispl_front_funcs *xendrm_front_funcs);
int xendrm_remove(struct platform_device *pdev);
bool xendrm_is_used(struct platform_device *pdev);

void xendrm_vtimer_restart_to(struct xendrm_du_device *xendrm_du, int index);

#endif /* __XEN_DRM_H*/

