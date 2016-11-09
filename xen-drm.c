/*
 *  Xen para-virtual DRM device
 *
 *  Based on
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

#include "xen-drm.h"

int xendrm_probe(struct platform_device *pdev)
{
	struct xendrm_plat_data *platdata;

	platdata = dev_get_platdata(&pdev->dev);
//	LOG0("Creating virtual DRM card %d", platdata->index);
	return 0;
}

int xendrm_remove(struct platform_device *pdev)
{
	return 0;
}
