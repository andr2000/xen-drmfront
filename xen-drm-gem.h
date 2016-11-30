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

dma_addr_t xendrm_gem_dumb_create(struct drm_file *file_priv,
	struct drm_device *dev, struct drm_mode_create_dumb *args);
int xendrm_gem_dumb_destroy(struct drm_file *file,
	struct drm_device *dev, uint32_t handle);
void xendrm_gem_free_object(struct drm_gem_object *obj);
int xendrm_gem_mmap(struct file *filp, struct vm_area_struct *vma);
int xendrm_gem_dumb_map_offset(struct drm_file *file_priv,
	struct drm_device *drm, u32 handle, u64 *offset);

#endif /* __XEN_DRM_GEM_H */
