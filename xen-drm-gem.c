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
#include <drm/drm_gem.h>

#include "xen-drm-gem.h"

#define to_xendrm_gem_obj(x) container_of(x, struct xendrm_gem_object, base)

struct xendrm_gem_object {
	struct drm_gem_object base;
	void *vaddr;
};

static struct xendrm_gem_object *xendrm_gem_create_obj(struct drm_device *dev,
	size_t size)
{
	struct xendrm_gem_object *xendrm_obj;
	struct drm_gem_object *gem_obj;
	int ret;

	xendrm_obj = kzalloc(sizeof(*xendrm_obj), GFP_KERNEL);
	if (!xendrm_obj)
		return ERR_PTR(-ENOMEM);
	gem_obj = &xendrm_obj->base;
	ret = drm_gem_object_init(dev, gem_obj, size);
	if (ret < 0)
		goto fail;
	ret = drm_gem_create_mmap_offset(gem_obj);
	if (ret < 0) {
		drm_gem_object_release(gem_obj);
		goto fail;
	}
	return xendrm_obj;
fail:
	kfree(xendrm_obj);
	return ERR_PTR(ret);
}

struct xendrm_gem_object *xendrm_gem_create(struct drm_device *dev, size_t size)
{
	struct xendrm_gem_object *xendrm_obj;
	int ret;

	size = round_up(size, PAGE_SIZE);
	xendrm_obj = xendrm_gem_create_obj(dev, size);
	if (IS_ERR(xendrm_obj))
		return xendrm_obj;
	xendrm_obj->vaddr = alloc_pages_exact(size, GFP_KERNEL);
	if (!xendrm_obj->vaddr) {
		dev_err(dev->dev, "failed to allocate buffer with size %zu\n",
			size);
		ret = -ENOMEM;
		goto error;
	}
	return xendrm_obj;

error:
	drm_gem_object_unreference_unlocked(&xendrm_obj->base);
	return ERR_PTR(ret);
}

static struct xendrm_gem_object *xendrm_gem_create_with_handle(
	struct drm_file *file_priv, struct drm_device *dev, size_t size,
	uint32_t *handle)
{
	struct xendrm_gem_object *xendrm_obj;
	struct drm_gem_object *gem_obj;
	int ret;

	xendrm_obj = xendrm_gem_create(dev, size);
	if (IS_ERR(xendrm_obj))
		return xendrm_obj;
	gem_obj = &xendrm_obj->base;
	ret = drm_gem_handle_create(file_priv, gem_obj, handle);
	/* drop reference from allocate - handle holds it now. */
	drm_gem_object_unreference_unlocked(gem_obj);
	if (ret < 0)
		return ERR_PTR(ret);
	return xendrm_obj;
}

void *xendrm_gem_dumb_create(struct drm_file *file_priv,
	struct drm_device *dev, struct drm_mode_create_dumb *args)
{
	args->pitch = DIV_ROUND_UP(args->width * args->bpp, 8);
	args->size = args->pitch * args->height;
	return xendrm_gem_create_with_handle(file_priv, dev, args->size,
		&args->handle);
}

int xendrm_gem_dumb_destroy(struct drm_file *file,
	struct drm_device *dev, uint32_t handle)
{
	return -EINVAL;
}

void xendrm_gem_free_object(struct drm_gem_object *obj)
{
}

int xendrm_gem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct drm_gem_object *gem_obj;
	int ret;

	ret = drm_gem_mmap(filp, vma);
	if (ret < 0)
		return ret;
	gem_obj = vma->vm_private_data;
	return -EINVAL;
}
