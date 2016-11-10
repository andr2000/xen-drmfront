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

#ifndef __XEN_DRM_GROUP_H_
#define __XEN_DRM_GROUP_H_

#include "xen-drm-plane.h"

struct xendrm_du_device;

/*
 * struct xendrm_du_group - CRTCs and planes group
 * @dev: the DU device
 * @index: group index
 * @use_count: number of users of the group (rcar_du_group_(get|put))
 * @used_crtcs: number of CRTCs currently in use
 * @planes: planes handled by the group
 * @need_restart: the group needs to be restarted due to a configuration change
 */
struct xendrm_du_group {
	struct xendrm_du_device *dev;
	unsigned int index;

	/* FIXME: support only 1 CRTC and 1 plane */
	unsigned int use_count;
	unsigned int used_crtcs;

	struct xendrm_du_plane planes[1];
	bool need_restart;
};

#endif /* __XEN_DRM_GROUP_H_ */
