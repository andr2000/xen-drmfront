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
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem.h>

#include <linux/dma-buf.h>
#include <linux/scatterlist.h>

#include "xen-drm-gem.h"

struct xen_gem_object {
	struct drm_gem_object base;
	size_t size;
	/* allocated by us */
	void *vaddr;
	/* imported */
	struct sg_table *sgt;
};

struct xen_fb {
	struct drm_framebuffer fb;
	struct xen_gem_object *xen_obj;
};

static inline struct xen_gem_object *to_xen_gem_obj(
	struct drm_gem_object *gem_obj)
{
	return container_of(gem_obj, struct xen_gem_object, base);
}

static inline struct xen_fb *to_xen_fb(struct drm_framebuffer *fb)
{
	return container_of(fb, struct xen_fb, fb);
}

static struct xen_gem_object *xendrm_gem_create_obj(struct drm_device *dev,
	size_t size)
{
	struct xen_gem_object *xen_obj;
	struct drm_gem_object *gem_obj;
	int ret;

	xen_obj = kzalloc(sizeof(*xen_obj), GFP_KERNEL);
	if (!xen_obj)
		return ERR_PTR(-ENOMEM);
	gem_obj = &xen_obj->base;
	ret = drm_gem_object_init(dev, gem_obj, size);
	if (ret)
		goto error;
	ret = drm_gem_create_mmap_offset(gem_obj);
	if (ret) {
		drm_gem_object_release(gem_obj);
		goto error;
	}
	return xen_obj;

error:
	kfree(xen_obj);
	return ERR_PTR(ret);
}

static struct xen_gem_object *xendrm_gem_create(struct drm_device *dev,
	size_t size)
{
	struct xen_gem_object *xen_obj;
	int ret;

	size = round_up(size, PAGE_SIZE);
	xen_obj = xendrm_gem_create_obj(dev, size);
	if (IS_ERR(xen_obj))
		return xen_obj;
	xen_obj->size = size;
	xen_obj->vaddr = alloc_pages_exact(size, GFP_KERNEL);
	if (!xen_obj->vaddr)
		ret = -ENOMEM;
		goto fail;
	return xen_obj;

fail:
	DRM_ERROR("Failed to allocate buffer with size %zu\n", size);
	drm_gem_object_unreference_unlocked(&xen_obj->base);
	return ERR_PTR(ret);
}

static struct xen_gem_object *xendrm_gem_create_with_handle(
	struct drm_file *file_priv, struct drm_device *dev,
	size_t size, uint32_t *handle)
{
	struct xen_gem_object *xen_obj;
	struct drm_gem_object *gem_obj;
	int ret;

	xen_obj = xendrm_gem_create(dev, size);
	if (IS_ERR(xen_obj))
		return xen_obj;
	gem_obj = &xen_obj->base;
	ret = drm_gem_handle_create(file_priv, gem_obj, handle);
	drm_gem_object_unreference_unlocked(gem_obj);
	if (ret)
		return ERR_PTR(ret);
	return xen_obj;
}

int xendrm_gem_dumb_create(struct drm_file *file_priv,
	struct drm_device *dev, struct drm_mode_create_dumb *args)
{
	struct xen_gem_object *xen_obj;

	args->pitch = DIV_ROUND_UP(args->width * args->bpp, 8);
	args->size = args->pitch * args->height;

	xen_obj = xendrm_gem_create_with_handle(file_priv, dev, args->size,
		&args->handle);
	return PTR_ERR_OR_ZERO(xen_obj);
}

void xendrm_gem_free_object(struct drm_gem_object *gem_obj)
{
	struct xen_gem_object *xen_obj = to_xen_gem_obj(gem_obj);

	if (xen_obj->vaddr)
		free_pages_exact(xen_obj->vaddr, xen_obj->size);
	else if (gem_obj->import_attach)
		drm_prime_gem_destroy(gem_obj, xen_obj->sgt);
	drm_gem_object_release(gem_obj);
	kfree(xen_obj);
}

struct sg_table *xendrm_gem_get_sg_table(struct drm_gem_object *gem_obj)
{
	struct xen_gem_object *xen_obj = to_xen_gem_obj(gem_obj);
	struct sg_table *sgt;
	int ret;

	if (!xen_obj->vaddr)
		return NULL;
	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return NULL;
	ret = sg_alloc_table(sgt, 1, GFP_KERNEL);
	if (unlikely(ret < 0))
		goto fail;
	sg_set_page(sgt->sgl, virt_to_page(xen_obj->vaddr), xen_obj->size, 0);
	return sgt;

fail:
	kfree(sgt);
	return NULL;
}

struct drm_gem_object *xendrm_gem_import_sg_table(struct drm_device *dev,
	struct dma_buf_attachment *attach, struct sg_table *sgt)
{
	struct xen_gem_object *xen_obj;

	xen_obj = xendrm_gem_create_obj(dev, attach->dmabuf->size);
	if (IS_ERR(xen_obj))
		return ERR_CAST(xen_obj);
	xen_obj->sgt = sgt;
	return &xen_obj->base;
}

static struct xen_fb *xendrm_gem_fb_alloc(struct drm_device *dev,
	const struct drm_mode_fb_cmd2 *mode_cmd,
	struct xen_gem_object *xen_obj,
	const struct drm_framebuffer_funcs *funcs)
{
	struct xen_fb *xen_fb;
	int ret;

	xen_fb = kzalloc(sizeof(*xen_fb), GFP_KERNEL);
	if (!xen_fb)
		return ERR_PTR(-ENOMEM);

	drm_helper_mode_fill_fb_struct(&xen_fb->fb, mode_cmd);
	xen_fb->xen_obj = xen_obj;
	ret = drm_framebuffer_init(dev, &xen_fb->fb, funcs);
	if (ret < 0) {
		DRM_ERROR("Failed to initialize framebuffer: %d\n", ret);
		kfree(xen_fb);
		return ERR_PTR(ret);
	}
	return xen_fb;
}

struct drm_framebuffer *xendrm_gem_fb_create_with_funcs(struct drm_device *dev,
	struct drm_file *file_priv, const struct drm_mode_fb_cmd2 *mode_cmd,
	const struct drm_framebuffer_funcs *funcs)
{
	struct xen_fb *xen_fb;
	struct xen_gem_object *xen_obj;
	struct drm_gem_object *gem_obj;
	unsigned int hsub;
	unsigned int vsub;
	unsigned int min_size;
	int ret;

	/* we do not support formats that require more than 1 plane */
	if (drm_format_num_planes(mode_cmd->pixel_format) != 1) {
		DRM_ERROR("Unsupported pixel format\n");
		return NULL;
	}
	hsub = drm_format_horz_chroma_subsampling(mode_cmd->pixel_format);
	vsub = drm_format_vert_chroma_subsampling(mode_cmd->pixel_format);

	gem_obj = drm_gem_object_lookup(file_priv, mode_cmd->handles[0]);
	if (!gem_obj) {
		DRM_ERROR("Failed to lookup GEM object\n");
		return ERR_PTR(-ENXIO);
	}

	min_size = (mode_cmd->height - 1) * mode_cmd->pitches[0] +
		mode_cmd->width *
		drm_format_plane_cpp(mode_cmd->pixel_format, 0) +
		mode_cmd->offsets[0];
	if (gem_obj->size < min_size) {
		drm_gem_object_unreference_unlocked(gem_obj);
		return ERR_PTR(-EINVAL);
	}
	xen_obj = to_xen_gem_obj(gem_obj);

	xen_fb = xendrm_gem_fb_alloc(dev, mode_cmd, xen_obj, funcs);
	if (IS_ERR(xen_fb)) {
		ret = PTR_ERR(xen_fb);
		goto fail;
	}
	return &xen_fb->fb;

fail:
	drm_gem_object_unreference_unlocked(gem_obj);
	return ERR_PTR(ret);
}

void xendrm_gem_fb_destroy(struct drm_framebuffer *fb)
{
	struct xen_fb *xen_fb = to_xen_fb(fb);

	if (xen_fb->xen_obj)
		drm_gem_object_unreference_unlocked(&xen_fb->xen_obj->base);
	drm_framebuffer_cleanup(fb);
	kfree(xen_fb);
}

int xendrm_gem_dumb_map_offset(struct drm_file *file_priv,
	struct drm_device *dev, uint32_t handle, uint64_t *offset)
{
	struct drm_gem_object *gem_obj;

	gem_obj = drm_gem_object_lookup(file_priv, handle);
	if (!gem_obj) {
		DRM_ERROR("Failed to lookup GEM object\n");
		return -EINVAL;
	}
	*offset = drm_vma_node_offset_addr(&gem_obj->vma_node);
	drm_gem_object_unreference_unlocked(gem_obj);
	return 0;
}

static int xendrm_gem_mmap_obj(struct xen_gem_object *xen_obj,
	struct vm_area_struct *vma)
{
	int ret;

	/*
	 * Clear the VM_PFNMAP flag that was set by drm_gem_mmap(), and set the
	 * vm_pgoff (used as a fake buffer offset by DRM) to 0 as we want to map
	 * the whole buffer.
	 */
	vma->vm_flags &= ~VM_PFNMAP;
	vma->vm_pgoff = 0;
	ret = dma_mmap_wc(xen_obj->base.dev->dev, vma, xen_obj->vaddr,
		virt_to_phys(xen_obj->vaddr), vma->vm_end - vma->vm_start);
	if (ret)
		drm_gem_vm_close(vma);
	return ret;
}

int xendrm_gem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct xen_gem_object *xen_obj;
	struct drm_gem_object *gem_obj;
	int ret;

	ret = drm_gem_mmap(filp, vma);
	if (ret)
		return ret;
	gem_obj = vma->vm_private_data;
	xen_obj = to_xen_gem_obj(gem_obj);
	return xendrm_gem_mmap_obj(xen_obj, vma);
}

void *xendrm_gem_prime_vmap(struct drm_gem_object *gem_obj)
{
	struct xen_gem_object *xen_obj = to_xen_gem_obj(gem_obj);

	return xen_obj->vaddr;
}

void xendrm_gem_prime_vunmap(struct drm_gem_object *gem_obj, void *vaddr)
{
	/* Nothing to do */
}

int xendrm_gem_prime_mmap(struct drm_gem_object *gem_obj,
	struct vm_area_struct *vma)
{
	struct xen_gem_object *xen_obj;
	int ret;

	ret = drm_gem_mmap_obj(gem_obj, gem_obj->size, vma);
	if (ret < 0)
		return ret;
	xen_obj = to_xen_gem_obj(gem_obj);
	return xendrm_gem_mmap_obj(xen_obj, vma);
}
