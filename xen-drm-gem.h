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

#ifndef __XEN_DRM_GEM_H
#define __XEN_DRM_GEM_H

#include <drm/drmP.h>

int xendrm_gem_dumb_create(struct drm_file *file_priv, struct drm_device *dev,
	struct drm_mode_create_dumb *args);
void xendrm_gem_free_object(struct drm_gem_object *gem_obj);
int xendrm_gem_dumb_map_offset(struct drm_file *file_priv,
	struct drm_device *dev, uint32_t handle, uint64_t *offset);

int xendrm_gem_mmap(struct file *filp, struct vm_area_struct *vma);

struct sg_table *xendrm_gem_get_sg_table(struct drm_gem_object *gem_obj);
struct drm_gem_object *xendrm_gem_import_sg_table(struct drm_device *dev,
	struct dma_buf_attachment *attach, struct sg_table *sgt);
void *xendrm_gem_prime_vmap(struct drm_gem_object *gem_obj);
void xendrm_gem_prime_vunmap(struct drm_gem_object *gem_obj, void *vaddr);
int xendrm_gem_prime_mmap(struct drm_gem_object *gem_obj,
	struct vm_area_struct *vma);

void xendrm_gem_fb_destroy(struct drm_framebuffer *fb);
struct drm_framebuffer *xendrm_gem_fb_create_with_funcs(struct drm_device *dev,
	struct drm_file *file_priv, const struct drm_mode_fb_cmd2 *mode_cmd,
	const struct drm_framebuffer_funcs *funcs);

#endif /* __XEN_DRM_GEM_H */
