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

#ifndef __XEN_DRM_PLANE_H_
#define __XEN_DRM_PLANE_H_

#include <drm/drmP.h>

struct xendrm_du_group;

struct xendrm_du_plane {
	struct drm_plane plane;
	struct xendrm_du_group *group;
};

int xendrm_du_planes_init(struct xendrm_du_group *rgrp);

#endif /* __XEN_DRM_PLANE_H_ */
