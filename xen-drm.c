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
#ifdef CONFIG_DRM_GEM_CMA_HELPER
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#endif

#include "xen-drm.h"
#include "xen-drm-front.h"
#include "xen-drm-kms.h"
#ifndef CONFIG_DRM_GEM_CMA_HELPER
#include "xen-drm-gem.h"
#endif
#include "xen-drm-logs.h"

int xendrm_enable_vblank(struct drm_device *dev, unsigned int pipe)
{
	struct xendrm_du_device *xendrm_du = dev->dev_private;

	if (unlikely(pipe >= xendrm_du->num_crtcs))
		return -EINVAL;
	xendrm_du_crtc_enable_vblank(&xendrm_du->crtcs[pipe], true);
	return 0;
}

void xendrm_disable_vblank(struct drm_device *dev, unsigned int pipe)
{
	struct xendrm_du_device *xendrm_du = dev->dev_private;

	if (unlikely(pipe >= xendrm_du->num_crtcs))
		return;
	xendrm_du_crtc_enable_vblank(&xendrm_du->crtcs[pipe], false);
}

struct xendrm_buf_ops {
	dma_addr_t (*dumb_create)(struct drm_file *file_priv,
		struct drm_device *dev, struct drm_mode_create_dumb *args);
	int (*dumb_destroy)(struct drm_file *file,
		struct drm_device *dev, uint32_t handle);
	void (*gem_free_object)(struct drm_gem_object *obj);
	int (*gem_dumb_map_offset)(struct drm_file *file_priv,
		struct drm_device *drm, u32 handle, u64 *offset);
	int (*mmap) (struct file *, struct vm_area_struct *);
};

#ifdef CONFIG_DRM_GEM_CMA_HELPER
static dma_addr_t xendrm_cma_dumb_create(struct drm_file *file_priv,
	struct drm_device *dev, struct drm_mode_create_dumb *args)
{
	struct drm_gem_object *gem_obj;
	struct drm_gem_cma_object *cma_obj;

	ret = drm_gem_cma_dumb_create(file_priv, dev, args);
	if (ret < 0)
		goto fail;
	gem_obj = drm_gem_object_lookup(file_priv, args->handle);
	if (!gem_obj)
		goto fail_destroy;
	drm_gem_object_unreference_unlocked(gem_obj);
	cma_obj = to_drm_gem_cma_obj(gem_obj);
	return cma_obj->paddr;

fail_destroy:
	drm_gem_dumb_destroy(file_priv, dev, args->handle);
fail:
	return 0;
}

static struct xendrm_buf_ops xendrm_buf_ops = {
	.dumb_create = xendrm_cma_dumb_create,
	.dumb_destroy = drm_gem_dumb_destroy,
	.gem_free_object = drm_gem_cma_free_object,
	.mmap = drm_gem_cma_mmap,
};
#else

static struct xendrm_buf_ops xendrm_buf_ops = {
	.dumb_create = xendrm_gem_dumb_create,
	.dumb_destroy = xendrm_gem_dumb_destroy,
	.gem_free_object = xendrm_gem_free_object,
	.gem_dumb_map_offset = xendrm_gem_dumb_map_offset,
	.mmap = xendrm_gem_mmap,
};
#endif

static int xendrm_dumb_create(struct drm_file *file_priv, struct drm_device *dev,
	struct drm_mode_create_dumb *args)
{
	struct xendrm_du_device *xendrm_du = dev->dev_private;
	dma_addr_t paddr;
	int ret;

	paddr = xendrm_buf_ops.dumb_create(file_priv, dev, args);
	if (!paddr) {
		ret = -ENOMEM;
		goto fail;
	}
	ret = xendrm_du->front_funcs->dbuf_create(
			xendrm_du->xdrv_info, args->handle, args->width,
			args->height, args->bpp, args->size, paddr);
	if (ret < 0)
		goto fail_destroy;
	return 0;

fail_destroy:
	xendrm_buf_ops.dumb_destroy(file_priv, dev, args->handle);
fail:
	DRM_ERROR("Failed to create dumb buffer, ret %d\n", ret);
	return ret;
}

static int xendrm_dumb_destroy(struct drm_file *file,
	struct drm_device *dev, uint32_t handle)
{
	struct xendrm_du_device *xendrm_du = dev->dev_private;

	xendrm_du->front_funcs->dbuf_destroy(xendrm_du->xdrv_info, handle);
	return xendrm_buf_ops.dumb_destroy(file, dev, handle);
}

void xendrm_free_object(struct drm_gem_object *obj)
{
	xendrm_buf_ops.gem_free_object(obj);
}

int xendrm_mmap(struct file *file, struct vm_area_struct *vma)
{
	int ret;

	ret = xendrm_buf_ops.mmap(file, vma);
	DRM_ERROR("xendrm_buf_ops.mmap %d\n", ret);
	return ret;
}

int xendrm_dumb_map_offset(struct drm_file *file_priv,
	struct drm_device *dev, uint32_t handle, uint64_t *offset)
{
	int ret;

	ret = xendrm_buf_ops.gem_dumb_map_offset(file_priv, dev, handle, offset);
	DRM_ERROR("xendrm_buf_ops.gem_dumb_map_offset %d\n", ret);
	return ret;
}

static void xendrm_on_page_flip(struct platform_device *pdev,
	int conn_idx, uint64_t fb_cookie)
{
	struct xendrm_du_device *xendrm_du = platform_get_drvdata(pdev);

	if (unlikely(conn_idx >= xendrm_du->num_crtcs))
		return;
	xendrm_du_crtc_on_page_flip(&xendrm_du->crtcs[conn_idx], fb_cookie);
}

static const struct file_operations xendrm_fops = {
	.owner          = THIS_MODULE,
	.open           = drm_open,
	.release        = drm_release,
	.unlocked_ioctl = drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = drm_compat_ioctl,
#endif
	.poll           = drm_poll,
	.read           = drm_read,
	.llseek         = no_llseek,
	.mmap           = xendrm_mmap,
};

const struct vm_operations_struct xendrm_gem_vm_ops = {
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static struct drm_driver xendrm_driver = {
	.driver_features           = DRIVER_GEM | DRIVER_MODESET |
	                             DRIVER_PRIME | DRIVER_ATOMIC,
	.get_vblank_counter        = drm_vblank_count,
	.enable_vblank             = xendrm_enable_vblank,
	.disable_vblank            = xendrm_disable_vblank,
	.get_vblank_counter        = drm_vblank_no_hw_counter,
	.gem_free_object           = xendrm_free_object,
	.gem_vm_ops                = &xendrm_gem_vm_ops,
	.prime_handle_to_fd        = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle        = drm_gem_prime_fd_to_handle,
	.gem_prime_import          = drm_gem_prime_import,
	.gem_prime_export          = drm_gem_prime_export,
#ifdef CONFIG_DRM_GEM_CMA_HELPER
	.gem_prime_get_sg_table    = drm_gem_cma_prime_get_sg_table,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_vmap            = drm_gem_cma_prime_vmap,
	.gem_prime_vunmap          = drm_gem_cma_prime_vunmap,
	.gem_prime_mmap            = drm_gem_cma_prime_mmap,
#else
	.dumb_map_offset           = xendrm_dumb_map_offset,
#endif
	.dumb_create               = xendrm_dumb_create,
	.dumb_destroy              = xendrm_dumb_destroy,
	.fops                      = &xendrm_fops,
	.name                      = "xendrm-du",
	.desc                      = "Xen PV DRM Display Unit",
	.date                      = "20161109",
	.major                     = 1,
	.minor                     = 0,
};

int xendrm_probe(struct platform_device *pdev,
	struct xendispl_front_funcs *xendrm_front_funcs)
{
	struct xendrm_plat_data *platdata;
	struct xendrm_du_device *xendrm_du;
	struct drm_device *ddev;
	int ret;

	platdata = dev_get_platdata(&pdev->dev);
	LOG0("Creating %s", xendrm_driver.desc);
	/* Allocate and initialize the DRM and xendrm device structures. */
	xendrm_du = devm_kzalloc(&pdev->dev, sizeof(*xendrm_du), GFP_KERNEL);
	if (!xendrm_du)
		return -ENOMEM;

	xendrm_du->front_funcs = xendrm_front_funcs;
	xendrm_du->front_funcs->on_page_flip = xendrm_on_page_flip;
	xendrm_du->xdrv_info = platdata->xdrv_info;

	ddev = drm_dev_alloc(&xendrm_driver, &pdev->dev);
	if (!ddev)
		return -ENOMEM;

	xendrm_du->ddev = ddev;
	/*
	 * FIXME: assume 1 CRTC and 1 Encoder per each connector
	 */
	xendrm_du->num_crtcs = platdata->num_connectors;
	xendrm_du->platdata = platdata;
	ddev->dev_private = xendrm_du;
	platform_set_drvdata(pdev, xendrm_du);

	ret = drm_vblank_init(ddev, xendrm_du->num_crtcs);
	if (ret < 0)
		goto fail;
	/* DRM/KMS objects */
	ret = xendrm_du_modeset_init(xendrm_du);
	if (ret < 0) {
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev,
				"failed to initialize DRM/KMS (%d)\n", ret);
		goto fail;
	}
	ddev->irq_enabled = 1;

	/* Register the DRM device with the core and the connectors,
	 * encoders, planes with sysfs.
	 */
	ret = drm_dev_register(ddev, 0);
	if (ret)
		goto fail;

	DRM_INFO("Initialized %s %d.%d.%d %s on minor %d\n",
		xendrm_driver.name, xendrm_driver.major,
		xendrm_driver.minor, xendrm_driver.patchlevel,
		xendrm_driver.date, ddev->primary->index);
	return 0;
fail:
	xendrm_remove(pdev);
	return ret;
}

int xendrm_remove(struct platform_device *pdev)
{
	struct xendrm_du_device *xendrm_du = platform_get_drvdata(pdev);
	struct drm_device *drm_dev = xendrm_du->ddev;

	drm_dev_unregister(drm_dev);
	xendrm_du_modeset_cleanup(xendrm_du);
	drm_vblank_cleanup(drm_dev);
	drm_dev_unref(drm_dev);
	return 0;
}
