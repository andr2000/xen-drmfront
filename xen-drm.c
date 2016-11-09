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
#include <drm/drm_gem_cma_helper.h>

#include "xen-drm.h"
#include "xen-drm-logs.h"

static int xendrm_unload(struct drm_device *dev)
{
	return 0;
}

static int xendrm_load(struct drm_device *dev, unsigned long flags)
{
	return 0;
}

void xendrm_preclose(struct drm_device *dev, struct drm_file *file)
{
}

void xendrm_postclose(struct drm_device *dev, struct drm_file *file)
{
}

void xendrm_lastclose(struct drm_device *dev)
{
}

int xendrm_enable_vblank(struct drm_device *dev, unsigned int pipe)
{
	return 0;
}

void xendrm_disable_vblank(struct drm_device *dev, unsigned int pipe)
{
}

int xendrm_dumb_create(struct drm_file *file_priv, struct drm_device *dev,
	struct drm_mode_create_dumb *args)
{
	return 0;
}

static const struct file_operations xendrm_fops = {
	.owner		= THIS_MODULE,
	.open		= drm_open,
	.release	= drm_release,
	.unlocked_ioctl	= drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= drm_compat_ioctl,
#endif
	.poll		= drm_poll,
	.read		= drm_read,
	.llseek		= no_llseek,
	.mmap		= drm_gem_cma_mmap,
};

void xendrm_gem_cma_free_object(struct drm_gem_object *obj)
{
}

static struct drm_driver xendrm_driver = {
	.driver_features           = (DRIVER_GEM | \
			DRIVER_MODESET | DRIVER_PRIME),
	.load                      = xendrm_load,
	.unload                    = xendrm_unload,
	.preclose                  = xendrm_preclose,
	.lastclose                 = xendrm_lastclose,
	.get_vblank_counter        = drm_vblank_count,
	.enable_vblank             = xendrm_enable_vblank,
	.disable_vblank            = xendrm_disable_vblank,
	.gem_free_object           = xendrm_gem_cma_free_object,
	.gem_vm_ops                = &drm_gem_cma_vm_ops,
	.prime_handle_to_fd        = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle        = drm_gem_prime_fd_to_handle,
	.gem_prime_import          = drm_gem_prime_import,
	.gem_prime_export          = drm_gem_prime_export,
	.gem_prime_get_sg_table    = drm_gem_cma_prime_get_sg_table,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_vmap            = drm_gem_cma_prime_vmap,
	.gem_prime_vunmap          = drm_gem_cma_prime_vunmap,
	.gem_prime_mmap            = drm_gem_cma_prime_mmap,
	.dumb_create               = xendrm_dumb_create,
	.dumb_map_offset           = drm_gem_cma_dumb_map_offset,
	.dumb_destroy              = drm_gem_dumb_destroy,
	.fops                      = &xendrm_fops,
	.name                      = "xendrm-du",
	.desc                      = "Xen PV DRM Display Unit",
	.date                      = "20161109",
	.major                     = 1,
	.minor                     = 0,
};

int xendrm_probe(struct platform_device *pdev)
{
	struct xendrm_plat_data *platdata;

	platdata = dev_get_platdata(&pdev->dev);
	struct drm_device *drm_dev;
	int ret;

	LOG0("Creating virtual DRM card %d", platdata->index);
	drm_dev = drm_dev_alloc(&xendrm_driver, &pdev->dev);
	if (!drm_dev)
		return -ENOMEM;

	drm_dev->platformdev = pdev;

	ret = drm_dev_register(drm_dev, 0);
	if (ret)
		goto err_free;

	DRM_INFO("Initialized %s %d.%d.%d %s on minor %d\n",
		xendrm_driver.name, xendrm_driver.major,
		xendrm_driver.minor, xendrm_driver.patchlevel,
		xendrm_driver.date, drm_dev->primary->index);
	return 0;

err_free:
	drm_dev_unref(drm_dev);
	return ret;
}

int xendrm_remove(struct platform_device *pdev)
{
	struct xendrm_plat_data *platdata;

	platdata = dev_get_platdata(&pdev->dev);
	return 0;
}
