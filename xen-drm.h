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

#define XENDRM_DU_MAX_CRTCS	4

/*
 * struct xendrm_du_output_routing - Output routing specification
 * @possible_crtcs: bitmask of possible CRTCs for the output
 * @encoder_type: DRM type of the internal encoder associated with the output
 * @port: device tree port number corresponding to this output route
 */
struct xendrm_du_output_routing {
	unsigned int possible_crtcs;
	unsigned int encoder_type;
	unsigned int port;
};

struct xendrm_du_device {
	struct device *dev;
	struct drm_device *ddev;
	struct drm_fbdev_cma *fbdev;

	unsigned int num_crtcs;
	struct xendrm_du_crtc crtcs[XENDRM_DU_MAX_CRTCS];

	struct {
		struct drm_property *alpha;
		struct drm_property *colorkey;
	} props;
};

struct xendrm_cfg_connector {
	char type[32];
	int id;
	int width;
	int height;
	char *xenstore_path;
};

struct xendrm_cfg_card {
	/* number of connectors in this configuration */
	int num_connectors;
	/* connector configurations */
	struct xendrm_cfg_connector *connectors;
};

struct xendrm_plat_data {
	int index;
	struct xdrv_info *xdrv_info;
	struct xendrm_cfg_card cfg_card;
};

int xendrm_probe(struct platform_device *pdev);
int xendrm_remove(struct platform_device *pdev);

#endif /* __XEN_DRM_H*/

