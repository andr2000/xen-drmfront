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

#include <video/videomode.h>

#include "xen-drm.h"

static const struct drm_encoder_funcs xendrm_drm_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

int xendrm_du_encoder_create(struct xendrm_du_device *xendrm_du,
	struct xendrm_du_crtc *du_crtc)
{
	struct drm_encoder *encoder = &du_crtc->encoder;
	int ret;

	/* only this CRTC w/o any clones */
	encoder->possible_crtcs = 1 << du_crtc->index;
	encoder->possible_clones = 0;
	ret = drm_encoder_init(xendrm_du->drm_dev, encoder,
		&xendrm_drm_encoder_funcs, DRM_MODE_ENCODER_VIRTUAL, NULL);
	if (ret < 0)
		return ret;
	return 0;
}

static enum drm_connector_status
xendrm_du_drm_connector_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

#define XENDRM_NUM_VIDEO_MODES	1

static const struct videomode xendrm_def_videomode = {
	.pixelclock = 60 * 1024 * 768,
	.hactive = 1024,
	.hfront_porch = 0,
	.hback_porch = 0,
	.hsync_len = 0,
	.vactive = 768,
	.vfront_porch = 0,
	.vback_porch = 0,
	.vsync_len = 0,
	.flags = 0,
};

#define to_xendrm_connector(e) \
	container_of(e, struct xendrm_du_connector, base)

static int xendrm_du_drm_connector_get_modes(struct drm_connector *connector)
{
	struct xendrm_du_connector *du_connector;
	struct drm_display_mode *mode;
	struct videomode videomode;
	int width, height;

	mode = drm_mode_create(connector->dev);
	if (!mode)
		return 0;
	videomode = xendrm_def_videomode;
	du_connector = to_xendrm_connector(connector);
	videomode.hactive = du_connector->width;
	videomode.vactive = du_connector->height;
	/* fixup the default value */
	width = videomode.hactive + videomode.hfront_porch +
		videomode.hback_porch + videomode.hsync_len;
	height = videomode.vactive + videomode.vfront_porch +
		videomode.vback_porch + videomode.vsync_len;
	videomode.pixelclock = width * height * 60;
	mode->type = DRM_MODE_TYPE_PREFERRED | DRM_MODE_TYPE_DRIVER;
	drm_display_mode_from_videomode(&videomode, mode);
	drm_mode_probed_add(connector, mode);
	return XENDRM_NUM_VIDEO_MODES;
}

static int xendrm_du_drm_connector_mode_valid(struct drm_connector *connector,
					    struct drm_display_mode *mode)
{
	struct xendrm_du_connector *du_connector =
		to_xendrm_connector(connector);
	if (mode->hdisplay != du_connector->width)
		return MODE_ERROR;
	if (mode->vdisplay != du_connector->height)
		return MODE_ERROR;
	return MODE_OK;
}

static const struct drm_connector_helper_funcs xendrm_du_connector_helper_funcs = {
	.get_modes = xendrm_du_drm_connector_get_modes,
	.mode_valid = xendrm_du_drm_connector_mode_valid,
};

static const struct drm_connector_funcs xendrm_du_drm_connector_funcs = {
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
	.destroy = drm_connector_cleanup,
	.detect = xendrm_du_drm_connector_detect,
	.dpms = drm_atomic_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.reset = drm_atomic_helper_connector_reset,
};

int xendrm_du_connector_create(struct xendrm_du_device *xendrm_du,
	struct xendrm_du_crtc *du_crtc, struct xendrm_cfg_connector *cfg)
{
	struct drm_encoder *encoder = &du_crtc->encoder;
	struct drm_connector *connector = &du_crtc->connector.base;
	struct drm_mode_config *mode_config = &xendrm_du->drm_dev->mode_config;
	int ret;

	du_crtc->connector.width = cfg->width;
	du_crtc->connector.height = cfg->height;
	du_crtc->connector.xen_id = cfg->id;
	ret = drm_connector_init(xendrm_du->drm_dev, connector,
		&xendrm_du_drm_connector_funcs, DRM_MODE_CONNECTOR_VIRTUAL);
	if (ret < 0)
		return ret;
	drm_connector_helper_add(connector, &xendrm_du_connector_helper_funcs);

	ret = drm_mode_connector_attach_encoder(connector, encoder);
	if (ret < 0)
		goto fail;

	drm_object_property_set_value(&connector->base,
		mode_config->dpms_property, DRM_MODE_DPMS_ON);
	return 0;

fail:
	drm_connector_cleanup(connector);
	return ret;
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

static int xendrm_du_plane_atomic_check(struct drm_plane *plane,
					  struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->fb;
	int i;

	if (!state->fb || !state->crtc)
		return 0;

	for (i = 0; i < ARRAY_SIZE(xendrm_du_drm_plane_formats); i++)
		if (fb->pixel_format == xendrm_du_drm_plane_formats[i])
			return 0;
	return -EINVAL;
}

static void xendrm_du_plane_atomic_disable(struct drm_plane *plane,
	struct drm_plane_state *old_state)
{
}

static void xendrm_du_plane_atomic_update(struct drm_plane *plane,
	struct drm_plane_state *old_state)
{
}

static const struct drm_plane_helper_funcs xendrm_du_plane_helper_funcs = {
	.atomic_check = xendrm_du_plane_atomic_check,
	.atomic_disable = xendrm_du_plane_atomic_disable,
	.atomic_update = xendrm_du_plane_atomic_update,
};

static const struct drm_plane_funcs xendrm_du_crtc_drm_plane_funcs = {
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
	.destroy = drm_plane_cleanup,
	.disable_plane = drm_atomic_helper_disable_plane,
	.reset = drm_atomic_helper_plane_reset,
	.update_plane = drm_atomic_helper_update_plane,
};

static struct drm_plane *xendrm_du_crtc_create_primary(
	struct xendrm_du_device *xendrm_du, struct xendrm_du_crtc *du_crtc)
{
	struct drm_plane *primary = &du_crtc->primary;
	int ret;

	ret = drm_universal_plane_init(xendrm_du->drm_dev, primary, 0,
		&xendrm_du_crtc_drm_plane_funcs,
		xendrm_du_drm_plane_formats,
		ARRAY_SIZE(xendrm_du_drm_plane_formats),
		DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret < 0) {
		return NULL;
	}
	drm_plane_helper_add(primary, &xendrm_du_plane_helper_funcs);
	return primary;
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

	du_crtc->index = index;
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
