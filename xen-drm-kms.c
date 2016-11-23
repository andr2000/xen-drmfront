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

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_cma_helper.h>

#include "xen-drm.h"
#include "xen-drm-front.h"
#include "xen-drm-kms.h"

static void xendrm_du_fb_destroy(struct drm_framebuffer *fb)
{
	struct xendrm_du_device *xendrm_du = fb->dev->dev_private;

	DRM_ERROR("%s fb->base.id %d\n", __FUNCTION__, fb->base.id);
	xendrm_du->front_funcs->fb_destroy(xendrm_du->xdrv_info, (uint64_t)fb);
	drm_fb_cma_destroy(fb);
}

static struct drm_framebuffer_funcs xendr_du_fb_funcs = {
	.destroy = xendrm_du_fb_destroy,
};

static struct drm_framebuffer *
xendrm_du_fb_create(struct drm_device *dev, struct drm_file *file_priv,
	const struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct xendrm_du_device *xendrm_du = dev->dev_private;
	static struct drm_framebuffer *fb;

	DRM_ERROR("%s drm_dev %p xendrm_du %p\n", __FUNCTION__, dev, xendrm_du);
	fb = drm_fb_cma_create_with_funcs(dev, file_priv,
		mode_cmd, &xendr_du_fb_funcs);
	if (!IS_ERR_OR_NULL(fb)) {
		/* FIXME: we take the first handle */
		if (xendrm_du->front_funcs->fb_create(
				xendrm_du->xdrv_info, mode_cmd->handles[0], (uint64_t)fb,
				fb->width, fb->height, fb->pixel_format) < 0) {
			drm_fb_cma_destroy(fb);
			fb = NULL;
		}
	}
	return fb;
}

static const struct drm_mode_config_funcs xendrm_du_mode_config_funcs = {
	.fb_create = xendrm_du_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

int xendrm_du_modeset_init(struct xendrm_du_device *xendrm_du)
{
	struct drm_device *drm_dev = xendrm_du->ddev;
	int i, ret;

	drm_mode_config_init(drm_dev);

	drm_dev->mode_config.min_width = 0;
	drm_dev->mode_config.min_height = 0;
	drm_dev->mode_config.max_width = 4095;
	drm_dev->mode_config.max_height = 2047;
	drm_dev->mode_config.funcs = &xendrm_du_mode_config_funcs;

	for (i = 0; i < xendrm_du->num_crtcs; i++) {
		struct xendrm_du_crtc *crtc;

		crtc = &xendrm_du->crtcs[i];
		ret = xendrm_du_crtc_create(xendrm_du, crtc, i);
		if (ret < 0)
			goto fail;
		ret = xendrm_du_encoder_create(xendrm_du, crtc);
		if (ret)
			goto fail;
		ret = xendrm_du_connector_create(xendrm_du, crtc,
			&xendrm_du->platdata->connectors[i]);
		if (ret)
			goto fail;
	}
	drm_mode_config_reset(drm_dev);

	return 0;
fail:
	drm_mode_config_cleanup(drm_dev);
	return ret;
}

void xendrm_du_modeset_cleanup(struct xendrm_du_device *xendrm_du)
{
	struct drm_device *drm_dev = xendrm_du->ddev;
	drm_mode_config_cleanup(drm_dev);
}
