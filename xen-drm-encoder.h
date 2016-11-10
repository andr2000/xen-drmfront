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

#ifndef __XEN_DRM_ENCODER_H_
#define __XEN_DRM_ENCODER_H_

#include <drm/drm_crtc.h>

#define to_xendrm_encoder(e) \
	container_of(e, struct xendrm_du_encoder, base)

struct xendrm_du_encoder {
	struct drm_encoder base;
//	enum xendrm_du_output output;
};

#endif /* __XEN_DRM_ENCODER_H_ */
