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
#include "xen-drm-front.h"

/* page flip complete event can be sent by either on back's
 * page flip completed event or atomic_flush, whatever is
 * _last_
 */
enum page_flip_event_senders {
	PF_EVT_SENDER_BACK,
	PF_EVT_SENDER_FLUSH,
	PF_EVT_SENDER_MAX,
};

static inline struct xendrm_du_connector *
to_xendrm_connector(struct drm_connector *connector)
{
	return container_of(connector, struct xendrm_du_connector, base);
}

static inline struct xendrm_du_crtc *
to_xendrm_crtc(struct drm_crtc *crtc)
{
	return container_of(crtc, struct xendrm_du_crtc, crtc);
}

static inline uint64_t xendrm_fb_to_cookie(struct drm_framebuffer *fb)
{
	return (uint64_t)fb;
}

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
	ret = drm_encoder_init(xendrm_du->ddev, encoder,
		&xendrm_drm_encoder_funcs, DRM_MODE_ENCODER_VIRTUAL, NULL);
	if (ret < 0)
		return ret;
	return 0;
}

static enum drm_connector_status
xendrm_du_drm_connector_detect(struct drm_connector *connector, bool force)
{
	/* TODO: check if on back disconnect connector_status_disconnected
	 * will help cleaning up
	 */
	return connector_status_connected;
}

#define XENDRM_NUM_VIDEO_MODES	1

static int xendrm_du_drm_connector_get_modes(struct drm_connector *connector)
{
	struct xendrm_du_connector *du_connector;
	struct drm_display_mode *mode;
	struct videomode videomode;
	int width, height;

	mode = drm_mode_create(connector->dev);
	if (!mode)
		return 0;
	memset(&videomode, 0, sizeof(videomode));
	du_connector = to_xendrm_connector(connector);
	videomode.hactive = du_connector->width;
	videomode.vactive = du_connector->height;
	width = videomode.hactive + videomode.hfront_porch +
		videomode.hback_porch + videomode.hsync_len;
	height = videomode.vactive + videomode.vfront_porch +
		videomode.vback_porch + videomode.vsync_len;
	videomode.pixelclock = width * height * XENDRM_CRTC_VREFRESH_HZ;
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
	struct drm_mode_config *mode_config = &xendrm_du->ddev->mode_config;
	int ret;

	du_crtc->connector.width = cfg->width;
	du_crtc->connector.height = cfg->height;
	ret = drm_connector_init(xendrm_du->ddev, connector,
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

static void xendrm_du_plane_atomic_update(struct drm_plane *plane,
	struct drm_plane_state *old_state)
{
}

static const struct drm_plane_helper_funcs xendrm_du_plane_helper_funcs = {
	.atomic_check = xendrm_du_plane_atomic_check,
	.atomic_update = xendrm_du_plane_atomic_update,
};

static const struct drm_plane_funcs xendrm_du_crtc_drm_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

static struct drm_plane *xendrm_du_crtc_create_primary(
	struct xendrm_du_device *xendrm_du, struct xendrm_du_crtc *du_crtc)
{
	struct drm_plane *primary = &du_crtc->primary;
	int ret;

	ret = drm_universal_plane_init(xendrm_du->ddev, primary, 0,
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
	struct xendrm_du_crtc *du_crtc)
{
	du_crtc->props.alpha = drm_property_create_range(xendrm_du->ddev,
		0, "alpha", 0, 255);
	if (!du_crtc->props.alpha)
		return -ENOMEM;
	return 0;
}

static inline bool xendrm_du_crtc_page_flip_pending(
	struct xendrm_du_crtc *du_crtc)
{
	struct drm_device *dev = du_crtc->crtc.dev;
	bool pending;
	unsigned long flags;

	spin_lock_irqsave(&dev->event_lock, flags);
	pending = du_crtc->pg_flip_event != NULL;
	spin_unlock_irqrestore(&dev->event_lock, flags);
	return pending;
}

static int xendrm_du_crtc_do_page_flip(struct drm_crtc *crtc,
	struct drm_framebuffer *fb, struct drm_pending_vblank_event *event,
	uint32_t drm_flags)
{
	struct xendrm_du_crtc *du_crtc = to_xendrm_crtc(crtc);
	struct drm_device *dev = du_crtc->crtc.dev;
	struct xendrm_du_device *xendrm_du;
	unsigned long flags;
	int ret;

	if (unlikely(xendrm_du_crtc_page_flip_pending(du_crtc))) {
		/* this can happen if user space doesn't honor
		 * page flip completed events
		 */
		DRM_ERROR("already have pending page flip\n");
		return -EBUSY;
	}

	/* There are 2 possible cases:
	 *   1. backend sends page flip completed before atomic_flush
	 *   2. backend is clumsy and sends event later than atomic_flush
	 * FIXME: drm_pending_vblank_event is not yet fully initialized
	 *   by the DRM core, so it cannot be used to send events right now
	 *   (see drm_ioctl), so use it as a placeholder which will not
	 *   allow concurrent flips
	 */
	spin_lock_irqsave(&dev->event_lock, flags);
	du_crtc->pg_flip_event = event;
	atomic_set(&du_crtc->pg_flip_senders, PF_EVT_SENDER_MAX);
	du_crtc->fb_cookie = xendrm_fb_to_cookie(fb);
	spin_unlock_irqrestore(&dev->event_lock, flags);

	xendrm_du = du_crtc->xendrm_du;

	ret = xendrm_du->front_funcs->page_flip(
		xendrm_du->xdrv_info, du_crtc->index, xendrm_fb_to_cookie(fb));
	if (unlikely(ret < 0))
		goto fail;

	/* at this stage back was armed and will send page flip event,
	 * so if we now fail then we have to drop incoming event
	 */
	ret = drm_atomic_helper_page_flip(crtc, fb, event, drm_flags);
	if (unlikely(ret < 0)) {
		du_crtc->fb_cookie = -1;
		goto fail;
	}

	/* restart page flip time-out counter */
	xendrm_vtimer_restart_to(xendrm_du, du_crtc->index);
	return 0;

fail:
	spin_lock_irqsave(&dev->event_lock, flags);
	atomic_set(&du_crtc->pg_flip_senders, 0);
	du_crtc->pg_flip_event = NULL;
	du_crtc->fb_cookie = xendrm_fb_to_cookie(NULL);
	spin_unlock_irqrestore(&dev->event_lock, flags);
	return ret;
}

static void xendrm_du_crtc_ntfy_page_flip_completed(
	struct xendrm_du_crtc *du_crtc)
{
	struct drm_device *dev = du_crtc->crtc.dev;
	unsigned long flags;

	if (unlikely(!xendrm_du_crtc_page_flip_pending(du_crtc)))
		return;
	spin_lock_irqsave(&dev->event_lock, flags);
	drm_crtc_send_vblank_event(&du_crtc->crtc, du_crtc->pg_flip_event);
	du_crtc->pg_flip_event = NULL;
	wake_up(&du_crtc->flip_wait);
	spin_unlock_irqrestore(&dev->event_lock, flags);
	drm_crtc_vblank_put(&du_crtc->crtc);
}

void xendrm_du_crtc_on_page_flip_done(struct xendrm_du_crtc *du_crtc,
	uint64_t fb_cookie)
{
	if (unlikely(du_crtc->fb_cookie != fb_cookie)) {
		DRM_ERROR("Drop page flip event: current %llx != %llx\n",
			du_crtc->fb_cookie, fb_cookie);
		return;
	}
	WARN_ON(atomic_read(&du_crtc->pg_flip_senders) == 0);
	if (atomic_dec_and_test(&du_crtc->pg_flip_senders))
		xendrm_du_crtc_ntfy_page_flip_completed(du_crtc);
}

void xendrm_du_crtc_on_page_flip_to(struct xendrm_du_crtc *du_crtc)
{
	if (xendrm_du_crtc_page_flip_pending(du_crtc)) {
		DRM_ERROR("Flip event timed-out, releasing\n");
		xendrm_du_crtc_ntfy_page_flip_completed(du_crtc);
		atomic_set(&du_crtc->pg_flip_senders, 0);
	}
}

static int xendrm_crtc_set_config(struct drm_mode_set *set)
{
	struct drm_crtc *crtc = set->crtc;
	struct xendrm_du_crtc *du_crtc = to_xendrm_crtc(crtc);
	struct xendrm_du_device *xendrm_du = du_crtc->xendrm_du;
	int ret;

	if (set->mode) {
		ret = xendrm_du->front_funcs->mode_set(du_crtc, set->x, set->y,
			set->fb->width, set->fb->height,
			set->fb->bits_per_pixel, (uint64_t)set->fb);
		if (ret < 0) {
			DRM_ERROR("Failed to set mode to back, ret %d\n", ret);
			return ret;
		}
	} else {
		ret = xendrm_du->front_funcs->mode_set(du_crtc,
			0, 0, 0, 0, 0, 0);
		if (ret < 0)
			DRM_ERROR("Failed to set mode to back, ret %d\n", ret);
		/* fall through - at least try to set mode locally */
	}
	return drm_atomic_helper_set_config(set);
}

static void xendrm_du_crtc_disable(struct drm_crtc *crtc)
{
	struct xendrm_du_crtc *du_crtc = to_xendrm_crtc(crtc);

	xendrm_vtimer_cancel_to(du_crtc->xendrm_du, du_crtc->index);
	if (wait_event_timeout(du_crtc->flip_wait,
			!xendrm_du_crtc_page_flip_pending(du_crtc),
			msecs_to_jiffies(XENDRM_CRTC_PFLIP_TO_MS)) == 0) {
		xendrm_du_crtc_ntfy_page_flip_completed(du_crtc);
	}
	drm_crtc_vblank_off(crtc);
}

static void xendrm_du_crtc_atomic_flush(struct drm_crtc *crtc,
	struct drm_crtc_state *old_crtc_state)
{
	struct xendrm_du_crtc *du_crtc = to_xendrm_crtc(crtc);
	struct drm_pending_vblank_event *event;
	struct drm_device *dev = crtc->dev;
	unsigned long flags;

	spin_lock_irqsave(&dev->event_lock, flags);
	event = crtc->state->event;
	crtc->state->event = NULL;
	if (event) {
		if (event->event.base.type == DRM_EVENT_FLIP_COMPLETE) {
			WARN_ON(drm_crtc_vblank_get(crtc) != 0);
			/* TODO: if the event set at .page_flip was initialized
			 * properly at this moment we do not need the
			 * assignment below
			 */
			du_crtc->pg_flip_event = event;
			WARN_ON(atomic_read(&du_crtc->pg_flip_senders) == 0);
			if (atomic_dec_and_test(&du_crtc->pg_flip_senders)) {
				spin_unlock_irqrestore(&dev->event_lock, flags);
				xendrm_du_crtc_ntfy_page_flip_completed(du_crtc);
				return;
			}
		} else {
			if (drm_crtc_vblank_get(crtc) == 0)
				drm_crtc_arm_vblank_event(crtc, event);
			else
				drm_crtc_send_vblank_event(crtc, event);
		}
	}
	spin_unlock_irqrestore(&dev->event_lock, flags);
}

static const struct drm_crtc_helper_funcs xendrm_du_drm_crtc_helper_funcs = {
	.atomic_flush = xendrm_du_crtc_atomic_flush,
	.enable = drm_crtc_vblank_on,
	.disable = xendrm_du_crtc_disable,
};

static const struct drm_crtc_funcs xendrm_du_drm_crtc_funcs = {
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.destroy = drm_crtc_cleanup,
	.page_flip = xendrm_du_crtc_do_page_flip,
	.reset = drm_atomic_helper_crtc_reset,
	.set_config = xendrm_crtc_set_config,
};

int xendrm_du_crtc_create(struct xendrm_du_device *xendrm_du,
	struct xendrm_du_crtc *du_crtc, unsigned int index)
{
	struct drm_plane *primary;
	int ret;

	memset(du_crtc, 0, sizeof(*du_crtc));
	du_crtc->xendrm_du = xendrm_du;
	du_crtc->index = index;
	init_waitqueue_head(&du_crtc->flip_wait);
	ret = xendrm_du_crtc_props_init(xendrm_du, du_crtc);
	if (ret < 0)
		return ret;
	primary = xendrm_du_crtc_create_primary(xendrm_du, du_crtc);
	if (!primary)
		return -ENOMEM;

	/* only primary plane, no cursor */
	ret = drm_crtc_init_with_planes(xendrm_du->ddev, &du_crtc->crtc,
		primary, NULL, &xendrm_du_drm_crtc_funcs, NULL);
	if (ret) {
		primary->funcs->destroy(primary);
		return ret;
	}
	drm_crtc_helper_add(&du_crtc->crtc, &xendrm_du_drm_crtc_helper_funcs);
	return 0;
}
