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

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_plane_helper.h>

#include "xen-drm.h"

static void xendrm_du_crtc_plane_destroy(struct drm_plane *plane)
{
	drm_plane_cleanup(plane);
	devm_kfree(plane->dev->dev, plane);
}

static int xendrm_du_crtc_props_init(struct xendrm_du_device *xendrm_du,
	struct xendrm_du_crtc *crtc)
{
	crtc->props.alpha = drm_property_create_range(xendrm_du->drm_dev,
		0, "alpha", 0, 255);
	if (!crtc->props.alpha)
		return -ENOMEM;
	return 0;
}

static const uint32_t xendrm_du_drm_plane_formats[] = {
	DRM_FORMAT_RGB565,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XRGB4444,
	DRM_FORMAT_ARGB4444,
	DRM_FORMAT_XRGB1555,
	DRM_FORMAT_ARGB1555,
	DRM_FORMAT_YUV422,
};

static const struct drm_plane_funcs xendrm_du_crtc_drm_plane_funcs = {
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
	.destroy = xendrm_du_crtc_plane_destroy,
	.disable_plane = drm_atomic_helper_disable_plane,
	.reset = drm_atomic_helper_plane_reset,
	.update_plane = drm_atomic_helper_update_plane,
};

static struct drm_plane *xendrm_du_crtc_create_primary(
	struct xendrm_du_device *xendrm_du, struct xendrm_du_crtc *crtc)
{
	struct drm_plane *primary;
	int ret;

	primary = devm_kzalloc(xendrm_du->dev, sizeof(*primary), GFP_KERNEL);
	if (!primary) {
		DRM_DEBUG_KMS("Failed to allocate primary plane\n");
		return NULL;
	}
	ret = drm_universal_plane_init(xendrm_du->drm_dev, primary, 0,
		&xendrm_du_crtc_drm_plane_funcs,
		xendrm_du_drm_plane_formats,
		ARRAY_SIZE(xendrm_du_drm_plane_formats),
		DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret < 0) {
		devm_kfree(xendrm_du->dev, primary);
		return NULL;
	}
	return primary;
}

static void xendrm_du_crtc_enable(struct drm_crtc *crtc)
{
	DRM_DEBUG_KMS("%s\n", __FUNCTION__);
}

static void xendrm_du_crtc_disable(struct drm_crtc *crtc)
{
	DRM_DEBUG_KMS("%s\n", __FUNCTION__);
}

static void xendrm_du_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	DRM_DEBUG_KMS("%s\n", __FUNCTION__);
}

static const struct drm_crtc_helper_funcs xendrm_du_drm_crtc_helper_funcs = {
	.enable = xendrm_du_crtc_enable,
	.disable = xendrm_du_crtc_disable,
	.mode_set_nofb = xendrm_du_crtc_mode_set_nofb,
};

static const struct drm_crtc_funcs xendrm_du_drm_crtc_funcs = {
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.destroy = drm_crtc_cleanup,
	.page_flip = drm_atomic_helper_page_flip,
	.reset = drm_atomic_helper_crtc_reset,
	.set_config = drm_atomic_helper_set_config,
};

int xendrm_du_crtc_create(struct xendrm_du_device *xendrm_du,
	struct xendrm_du_crtc *du_crtc, unsigned int index)
{
	struct drm_plane *primary;
	int ret;

	ret = xendrm_du_crtc_props_init(xendrm_du, du_crtc);
	if (ret < 0)
		return ret;
	primary = xendrm_du_crtc_create_primary(xendrm_du, du_crtc);
	if (!primary)
		return -ENOMEM;
	/* only primary plane, no cursor */
	ret = drm_crtc_init_with_planes(xendrm_du->drm_dev, &du_crtc->crtc,
		primary, NULL, &xendrm_du_drm_crtc_funcs, NULL);
	if (ret) {
		primary->funcs->destroy(primary);
		return ret;
	}
	drm_crtc_helper_add(&du_crtc->crtc, &xendrm_du_drm_crtc_helper_funcs);
	return 0;
}
