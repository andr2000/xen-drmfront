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

struct xendrm_du_device;

struct xendrm_du_crtc {
	int index;
	struct drm_crtc crtc;
	struct drm_fbdev_cma *fbdev;
	struct {
		struct drm_property *alpha;
	} props;
};

int xendrm_du_crtc_create(struct xendrm_du_device *xendrm_du,
	struct xendrm_du_crtc *du_crtc, unsigned int index);

#endif /* __XEN_DRM_CRTC_H_ */
