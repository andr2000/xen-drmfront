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

struct xdrv_info;
struct platform_device;
struct xendrm_du_crtc;

struct xendrm_front_funcs {
	int (*mode_set)(struct xendrm_du_crtc *du_crtc, uint32_t x, uint32_t y,
		uint32_t width, uint32_t height, uint32_t bpp, uint32_t fb_id);
	int (*dumb_create)(struct xdrv_info *drv_info, uint32_t handle, uint32_t width,
		uint32_t height, uint32_t bpp, uint64_t size, void *vaddr);
	int (*dumb_destroy)(struct xdrv_info *drv_info, uint32_t handle);
	int (*fb_create)(struct xdrv_info *drv_info, uint32_t handle, uint32_t fb_id,
		uint32_t width, uint32_t height, uint32_t pixel_format);
	int (*fb_destroy)(struct xdrv_info *drv_info, uint32_t fb_id);
	int (*page_flip)(struct xdrv_info *drv_info, int crtc_id, uint32_t fb_id);
	/* CAUTION! this is called with a spin_lock held! */
	void (*on_page_flip)(struct platform_device *pdev, int crtc_id, uint32_t fb_id);
};

#endif /* __XEN_DRM_FRONT_H_ */
