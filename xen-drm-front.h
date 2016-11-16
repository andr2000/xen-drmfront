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

#ifndef __XEN_DRM_FRONT_H_
#define __XEN_DRM_FRONT_H_

struct platform_device;
struct xendrm_du_crtc;
struct drm_gem_object;
struct drm_framebuffer;

struct xendrm_front_funcs {
	int (*mode_set)(struct xendrm_du_crtc *du_crtc);
	int (*dumb_create)(struct platform_device *pdev, struct drm_gem_object *gem_obj);
	int (*dumb_destroy)(struct platform_device *pdev, struct drm_gem_object *gem_obj);
	int (*fb_create)(struct platform_device *pdev, struct drm_framebuffer *fb);
	int (*fb_destroy)(struct platform_device *pdev, struct drm_framebuffer *fb);
	int (*page_flip)(struct platform_device *pdev, int crtc_id, int fb_id);
	/* CAUTION! this is called with a spin_lock held! */
	void (*on_page_flip)(struct platform_device *pdev, int fb_id);
};

#endif /* __XEN_DRM_FRONT_H_ */
