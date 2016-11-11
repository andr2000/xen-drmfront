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

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_mode.h>

#include "xen-drm.h"
#include "xen-drm-encoder.h"
#include "xen-drm-kms.h"

static struct drm_framebuffer *
xendrm_fb_create(struct drm_device *dev, struct drm_file *file_priv,
	struct drm_mode_fb_cmd2 *mode_cmd)
{
	return NULL;
}

static void xendrm_output_poll_changed(struct drm_device *dev)
{
}

/* FIXME: needs update if DRIVER_ATOMIC is supported */
static const struct drm_mode_config_funcs xendrm_du_mode_config_funcs = {
	.fb_create = xendrm_fb_create,
	.output_poll_changed = xendrm_output_poll_changed,
};

static int xendrm_du_encoders_init(struct xendrm_du_device *xendrm_du)
{
	return 0;
}

static int xendrm_du_properties_init(struct xendrm_du_device *xendrm_du)
{
	xendrm_du->props.alpha =
		drm_property_create_range(xendrm_du->ddev, 0, "alpha", 0, 255);
	if (xendrm_du->props.alpha == NULL)
		return -ENOMEM;

	/* The color key is expressed as an RGB888 triplet stored in a 32-bit
	 * integer in XRGB8888 format. Bit 24 is used as a flag to disable (0)
	 * or enable source color keying (1).
	 */
	xendrm_du->props.colorkey =
		drm_property_create_range(xendrm_du->ddev, 0, "colorkey",
					  0, 0x01ffffff);
	if (xendrm_du->props.colorkey == NULL)
		return -ENOMEM;

	return 0;
}

int xendrm_du_modeset_init(struct xendrm_du_device *xendrm_du)
{
	struct drm_device *dev = xendrm_du->ddev;
	struct drm_encoder *encoder;
	struct drm_fbdev_cma *fbdev;
	unsigned int num_encoders;
	unsigned int i;
	int ret;

	drm_mode_config_init(dev);

	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;
	dev->mode_config.max_width = 4095;
	dev->mode_config.max_height = 2047;
	dev->mode_config.funcs = &xendrm_du_mode_config_funcs;

	ret = xendrm_du_properties_init(xendrm_du);
	if (ret < 0)
		return ret;

	/* Create the CRTCs. */
	for (i = 0; i < xendrm_du->num_crtcs; ++i) {
		ret = xendrm_du_crtc_create(i);
		if (ret < 0)
			return ret;
	}

	/* Initialize encoder. */
	ret = xendrm_du_encoders_init(xendrm_du);
	if (ret < 0)
		return ret;

	if (ret == 0) {
		dev_err(xendrm_du->dev,
			"error: no encoder could be initialized\n");
		return -EINVAL;
	}

	num_encoders = ret;

	/* Set the possible CRTCs and possible clones. There's always at least
	 * one way for all encoders to clone each other, set all bits in the
	 * possible clones field.
	 */
#if 0
	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head) {
		struct xendrm_du_encoder *renc = to_xendrm_encoder(encoder);
		const struct xendrm_du_output_routing *route =
			&xendrm_du->info->routes[renc->output];

		encoder->possible_crtcs = route->possible_crtcs;
		encoder->possible_clones = (1 << num_encoders) - 1;
	}
#endif
	drm_mode_config_reset(dev);

	drm_kms_helper_poll_init(dev);

	if (dev->mode_config.num_connector) {
		fbdev = drm_fbdev_cma_init(dev, 32, dev->mode_config.num_crtc,
					   dev->mode_config.num_connector);
		if (IS_ERR(fbdev))
			return PTR_ERR(fbdev);

		xendrm_du->fbdev = fbdev;
	} else {
		dev_info(xendrm_du->dev,
			 "no connector found, disabling fbdev emulation\n");
	}
	return 0;
}
