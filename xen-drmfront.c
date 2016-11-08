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

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/vmalloc.h>

#include <asm/xen/hypervisor.h>
#include <xen/xen.h>
#include <xen/platform_pci.h>
#include <xen/xenbus.h>
#include <xen/events.h>
#include <xen/grant_table.h>
#include <xen/interface/io/ring.h>
#include <xen/interface/io/drmif_linux.h>

#define GRANT_INVALID_REF	0
/* timeout in ms to wait for backend to respond */
#define VDRM_WAIT_BACK_MS	5000

enum xdrv_evtchnl_state {
	EVTCHNL_STATE_DISCONNECTED,
	EVTCHNL_STATE_CONNECTED,
	EVTCHNL_STATE_SUSPENDED,
};

struct xdrv_evtchnl_info {
	struct xdrv_info *drv_info;
	struct xen_drmif_front_ring ring;
	int ring_ref;
	int port;
	int irq;
	struct completion completion;
	/* state of the event channel */
	enum xdrv_evtchnl_state state;
	/* latest response status and id */
	int resp_status;
	uint16_t resp_id;
};

struct xdrv_shared_buffer_info {
	int num_grefs;
	grant_ref_t *grefs;
	unsigned char *vdirectory;
	unsigned char *vbuffer;
	size_t vbuffer_sz;
};

struct xdrv_info {
	struct xenbus_device *xb_dev;
	spinlock_t io_lock;
	struct mutex mutex;
	bool ddrv_registered;
	/* array of virtual DRM platform devices */
	struct platform_device **ddrv_devs;

	int num_evt_channels;
	struct xdrv_evtchnl_info *evtchnls;
	int cfg_num_cards;
	struct ddev_plat_data *cfg_plat_data;
};

struct ddev_plat_data {
	int index;
	struct xdrv_info *xdrv_info;
};

struct DRMIF_TO_KERN_ERROR {
	int drmif;
	int kern;
};
static struct DRMIF_TO_KERN_ERROR drmif_kern_error_codes[] = {
	{ .drmif = XENDRM_RSP_OKAY,     .kern = 0 },
	{ .drmif = XENDRM_RSP_ERROR,    .kern = EIO },
};

static int drmif_to_kern_error(int drmif_err)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(drmif_kern_error_codes); i++)
		if (drmif_kern_error_codes[i].drmif == drmif_err)
			return -drmif_kern_error_codes[i].kern;
	return -EIO;
}

static int ddrv_probe(struct platform_device *pdev)
{
	struct ddev_plat_data *platdata;

	platdata = dev_get_platdata(&pdev->dev);
	dev_dbg(&pdev->dev, "Creating virtual DRM card %d", platdata->index);
	return 0;
}

static int ddrv_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver ddrv_info = {
	.probe		= ddrv_probe,
	.remove		= ddrv_remove,
	.driver		= {
		.name	= XENDRM_DRIVER_NAME,
	},
};

static void ddrv_cleanup(struct xdrv_info *drv_info)
{
	int i;

	if (!drv_info->ddrv_registered)
		return;
	if (drv_info->ddrv_devs) {
		for (i = 0; i < drv_info->cfg_num_cards; i++) {
			struct platform_device *ddrv_dev;

			ddrv_dev = drv_info->ddrv_devs[i];
			if (ddrv_dev)
				platform_device_unregister(ddrv_dev);
		}
	}
	platform_driver_unregister(&ddrv_info);
	drv_info->ddrv_registered = false;
}

static int ddrv_init(struct xdrv_info *drv_info)
{
	int i, num_cards, ret;

	ret = platform_driver_register(&ddrv_info);
	if (ret < 0)
		return ret;
	drv_info->ddrv_registered = true;

	num_cards = drv_info->cfg_num_cards;
	drv_info->ddrv_devs = devm_kzalloc(&drv_info->xb_dev->dev,
			sizeof(drv_info->ddrv_devs[0]) * num_cards,
			GFP_KERNEL);
	if (!drv_info->ddrv_devs)
		goto fail;
	for (i = 0; i < num_cards; i++) {
#if 0
		struct platform_device *ddrv_dev;
		struct ddev_plat_data *platdata;

		platdata = &drv_info->cfg_plat_data[i];
		/* pass card configuration via platform data */
		ddrv_dev = platform_device_register_data(NULL,
			XENDRM_DRIVER_NAME,
			platdata->index, platdata,
			sizeof(*platdata));
		drv_info->ddrv_devs[i] = ddrv_dev;
		if (IS_ERR(ddrv_dev)) {
			drv_info->ddrv_devs[i] = NULL;
			goto fail;
		}
#endif
	}
	return 0;

fail:
	dev_err(&drv_info->xb_dev->dev, "Failed to register DRM driver");
	ddrv_cleanup(drv_info);
	return -ENODEV;
}

static irqreturn_t xdrv_evtchnl_interrupt(int irq, void *dev_id)
{
	struct xdrv_evtchnl_info *channel = dev_id;
	struct xdrv_info *drv_info = channel->drv_info;
	struct xendrm_resp *resp;
	RING_IDX i, rp;
	unsigned long flags;

	spin_lock_irqsave(&drv_info->io_lock, flags);
	if (unlikely(channel->state != EVTCHNL_STATE_CONNECTED))
		goto out;

 again:
	rp = channel->ring.sring->rsp_prod;
	rmb(); /* Ensure we see queued responses up to 'rp'. */

	for (i = channel->ring.rsp_cons; i != rp; i++) {
		resp = RING_GET_RESPONSE(&channel->ring, i);
		if (resp->u.data.id != channel->resp_id)
			continue;
		switch (resp->u.data.operation) {
		case XENDRM_OP_OPEN:
		case XENDRM_OP_CLOSE:
			channel->resp_status = resp->u.data.status;
			complete(&channel->completion);
			break;
		default:
			dev_err(&drv_info->xb_dev->dev,
				"Operation %d is not supported",
				resp->u.data.operation);
			break;
		}
	}

	channel->ring.rsp_cons = i;

	if (i != channel->ring.req_prod_pvt) {
		int more_to_do;

		RING_FINAL_CHECK_FOR_RESPONSES(&channel->ring, more_to_do);
		if (more_to_do)
			goto again;
	} else
		channel->ring.sring->rsp_event = i + 1;

out:
	spin_unlock_irqrestore(&drv_info->io_lock, flags);
	return IRQ_HANDLED;
}

static void xdrv_evtchnl_free(struct xdrv_info *drv_info,
		struct xdrv_evtchnl_info *channel)
{
	if (!channel->ring.sring)
		return;
	channel->state = EVTCHNL_STATE_DISCONNECTED;
	/* release all who still waits for response if any */
	channel->resp_status = -XENDRM_RSP_ERROR;
	complete_all(&channel->completion);
	if (channel->irq)
		unbind_from_irqhandler(channel->irq, channel);
	channel->irq = 0;
	if (channel->port)
		xenbus_free_evtchn(drv_info->xb_dev, channel->port);
	channel->port = 0;
	/* End access and free the pages */
	if (channel->ring_ref != GRANT_INVALID_REF)
		gnttab_end_foreign_access(channel->ring_ref, 0,
			(unsigned long)channel->ring.sring);
	channel->ring_ref = GRANT_INVALID_REF;
	channel->ring.sring = NULL;
}

static void xdrv_evtchnl_free_all(struct xdrv_info *drv_info)
{
	int i;

	if (!drv_info->evtchnls)
		return;
	for (i = 0; i < drv_info->num_evt_channels; i++)
		xdrv_evtchnl_free(drv_info,
			&drv_info->evtchnls[i]);
	devm_kfree(&drv_info->xb_dev->dev, drv_info->evtchnls);
	drv_info->evtchnls = NULL;
}

static int xdrv_evtchnl_alloc(struct xdrv_info *drv_info,
		struct xdrv_evtchnl_info *evt_channel)
{
	struct xenbus_device *xb_dev = drv_info->xb_dev;
	struct xen_drmif_sring *sring;
	grant_ref_t gref;
	int ret;

	evt_channel->drv_info = drv_info;
	init_completion(&evt_channel->completion);
	evt_channel->state = EVTCHNL_STATE_DISCONNECTED;
	evt_channel->ring_ref = GRANT_INVALID_REF;
	evt_channel->ring.sring = NULL;
	evt_channel->port = 0;
	evt_channel->irq = 0;
	sring = (struct xen_drmif_sring *)get_zeroed_page(
		GFP_NOIO | __GFP_HIGH);
	if (!sring) {
		ret = -ENOMEM;
		goto fail;
	}
	SHARED_RING_INIT(sring);
	/* TODO: use XC_PAGE_SIZE */
	FRONT_RING_INIT(&evt_channel->ring, sring, PAGE_SIZE);

	ret = xenbus_grant_ring(xb_dev, sring, 1, &gref);
	if (ret < 0)
		goto fail;
	evt_channel->ring_ref = gref;

	ret = xenbus_alloc_evtchn(xb_dev, &evt_channel->port);
	if (ret < 0)
		goto fail;

	ret = bind_evtchn_to_irqhandler(evt_channel->port,
		xdrv_evtchnl_interrupt,
		0, xb_dev->devicetype, evt_channel);

	if (ret < 0)
		goto fail;
	evt_channel->irq = ret;
	return 0;

fail:
	dev_err(&xb_dev->dev, "Failed to allocate ring with err %d", ret);
	return ret;
}

static int xdrv_evtchnl_create(struct xdrv_info *drv_info,
		struct xdrv_evtchnl_info *evt_channel,
		const char *path)
{
	const char *message;
	int ret;

	/* allocate and open control channel */
	ret = xdrv_evtchnl_alloc(drv_info, evt_channel);
	if (ret < 0) {
		message = "allocating event channel";
		goto fail;
	}
	/* Write control channel ring reference */
	ret = xenbus_printf(XBT_NIL, path, XENDRM_FIELD_RING_REF, "%u",
			evt_channel->ring_ref);
	if (ret < 0) {
		message = "writing " XENDRM_FIELD_RING_REF;
		goto fail;
	}

	ret = xenbus_printf(XBT_NIL, path, XENDRM_FIELD_EVT_CHNL, "%u",
		evt_channel->port);
	if (ret < 0) {
		message = "writing " XENDRM_FIELD_EVT_CHNL;
		goto fail;
	}
	return 0;

fail:
	dev_err(&drv_info->xb_dev->dev, "Error %s with err %d", message, ret);
	return ret;
}

static inline void xdrv_evtchnl_flush(
		struct xdrv_evtchnl_info *channel)
{
	int notify;

	channel->ring.req_prod_pvt++;
	RING_PUSH_REQUESTS_AND_CHECK_NOTIFY(&channel->ring, notify);
	if (notify)
		notify_remote_via_irq(channel->irq);
}

static int xdrv_evtchnl_create_all(struct xdrv_info *drv_info,
		int num_streams)
{
	int ret, i;

	drv_info->evtchnls = devm_kcalloc(&drv_info->xb_dev->dev,
		num_streams, sizeof(struct xdrv_evtchnl_info),
		GFP_KERNEL);
	if (!drv_info->evtchnls) {
		ret = -ENOMEM;
		goto fail;
	}
	for (i = 0; i < drv_info->cfg_num_cards; i++) {
		struct ddev_plat_data *plat_data;

		//plat_data = &drv_info->cfg_plat_data[c];
		ret = xdrv_evtchnl_create(drv_info,
			&drv_info->evtchnls[i], "/local/");
		if (ret < 0)
			goto fail;
	}
	drv_info->num_evt_channels = num_streams;
	return 0;
fail:
	xdrv_evtchnl_free_all(drv_info);
	return ret;
}

/* get number of nodes under the path to get number of
 * cards configured or number of devices within the card
 */
static char **xdrv_cfg_get_num_nodes(const char *path, const char *node,
		int *num_entries)
{
	char **result;

	result = xenbus_directory(XBT_NIL, path, node, num_entries);
	if (IS_ERR(result)) {
		*num_entries = 0;
		return NULL;
	}
	return result;
}

static void xdrv_remove_internal(struct xdrv_info *drv_info)
{
	ddrv_cleanup(drv_info);
	xdrv_evtchnl_free_all(drv_info);
}

static int xdrv_probe(struct xenbus_device *xb_dev,
	const struct xenbus_device_id *id)
{
	struct xdrv_info *drv_info;
	int ret;

	drv_info = devm_kzalloc(&xb_dev->dev, sizeof(*drv_info), GFP_KERNEL);
	if (!drv_info) {
		ret = -ENOMEM;
		goto fail;
	}

	xenbus_switch_state(xb_dev, XenbusStateInitialising);

	drv_info->xb_dev = xb_dev;
	spin_lock_init(&drv_info->io_lock);
	mutex_init(&drv_info->mutex);
	drv_info->ddrv_registered = false;
	dev_set_drvdata(&xb_dev->dev, drv_info);
	return 0;
fail:
	xenbus_dev_fatal(xb_dev, ret, "allocating device memory");
	return ret;
}

static int xdrv_remove(struct xenbus_device *dev)
{
	struct xdrv_info *drv_info = dev_get_drvdata(&dev->dev);

	mutex_lock(&drv_info->mutex);
	xdrv_remove_internal(drv_info);
	mutex_unlock(&drv_info->mutex);
	xenbus_switch_state(dev, XenbusStateClosed);
	return 0;
}

static int xdrv_resume(struct xenbus_device *dev)
{
	return 0;
}

static grant_ref_t xdrv_sh_buf_get_dir_start(
	struct xdrv_shared_buffer_info *buf)
{
	if (!buf->grefs)
		return GRANT_INVALID_REF;
	return buf->grefs[0];
}

static void xdrv_sh_buf_clear(struct xdrv_shared_buffer_info *buf)
{
	buf->num_grefs = 0;
	buf->grefs = NULL;
	buf->vdirectory = NULL;
	buf->vbuffer = NULL;
	buf->vbuffer_sz = 0;
}

static void xdrv_sh_buf_free(struct xdrv_shared_buffer_info *buf)
{
	int i;

	if (buf->grefs) {
		for (i = 0; i < buf->num_grefs; i++)
			if (buf->grefs[i] != GRANT_INVALID_REF)
				gnttab_end_foreign_access(buf->grefs[i],
						0, 0UL);
		kfree(buf->grefs);
	}
	if (buf->vdirectory)
		vfree(buf->vdirectory);
	if (buf->vbuffer)
		vfree(buf->vbuffer);
	xdrv_sh_buf_clear(buf);
}

void xdrv_sh_buf_fill_page_dir(struct xdrv_shared_buffer_info *buf,
		int num_pages_dir)
{
	struct xendrm_page_directory *page_dir;
	unsigned char *ptr;
	int i, cur_gref, grefs_left, num_grefs_per_page, to_copy;

	ptr = buf->vdirectory;
	grefs_left = buf->num_grefs - num_pages_dir;
	num_grefs_per_page = (PAGE_SIZE - sizeof(
		struct xendrm_page_directory)) / sizeof(grant_ref_t);
	/* skip grefs at start, they are for pages granted for the directory */
	cur_gref = num_pages_dir;
	for (i = 0; i < num_pages_dir; i++) {
		page_dir = (struct xendrm_page_directory *)ptr;
		if (grefs_left <= num_grefs_per_page) {
			to_copy = grefs_left;
			page_dir->num_grefs = to_copy;
			page_dir->gref_dir_next_page = GRANT_INVALID_REF;
		} else {
			to_copy = num_grefs_per_page;
			page_dir->num_grefs = to_copy;
			page_dir->gref_dir_next_page = buf->grefs[i + 1];
		}
		memcpy(&page_dir->gref, &buf->grefs[cur_gref],
			to_copy * sizeof(grant_ref_t));
		ptr += PAGE_SIZE;
		grefs_left -= to_copy;
		cur_gref += to_copy;
	}
}

int xdrv_sh_buf_grant_refs(struct xenbus_device *xb_dev,
	struct xdrv_shared_buffer_info *buf,
	int num_pages_dir, int num_pages_vbuffer, int num_grefs)
{
	grant_ref_t priv_gref_head;
	int ret, i, j, cur_ref;
	int otherend_id;

	ret = gnttab_alloc_grant_references(num_grefs, &priv_gref_head);
	if (ret)
		return ret;
	buf->num_grefs = num_grefs;
	otherend_id = xb_dev->otherend_id;
	j = 0;
	for (i = 0; i < num_pages_dir; i++) {
		cur_ref = gnttab_claim_grant_reference(&priv_gref_head);
		if (cur_ref < 0)
			return cur_ref;
		gnttab_grant_foreign_access_ref(cur_ref, otherend_id,
			xen_page_to_gfn(vmalloc_to_page(buf->vdirectory +
				PAGE_SIZE * i)), 0);
		buf->grefs[j++] = cur_ref;
	}
	for (i = 0; i < num_pages_vbuffer; i++) {
		cur_ref = gnttab_claim_grant_reference(&priv_gref_head);
		if (cur_ref < 0)
			return cur_ref;
		gnttab_grant_foreign_access_ref(cur_ref, otherend_id,
			xen_page_to_gfn(vmalloc_to_page(buf->vbuffer +
				PAGE_SIZE * i)), 0);
		buf->grefs[j++] = cur_ref;
	}
	gnttab_free_grant_references(priv_gref_head);
	xdrv_sh_buf_fill_page_dir(buf, num_pages_dir);
	return 0;
}

int xdrv_sh_buf_alloc_buffers(struct xdrv_shared_buffer_info *buf,
		int num_pages_dir, int num_pages_vbuffer,
		int num_grefs)
{
	/* TODO: use XC_PAGE_SIZE */
	buf->grefs = kcalloc(num_grefs, sizeof(*buf->grefs), GFP_KERNEL);
	if (!buf->grefs)
		return -ENOMEM;
	buf->vdirectory = vmalloc(num_pages_dir * PAGE_SIZE);
	if (!buf->vdirectory)
		return -ENOMEM;
	buf->vbuffer_sz = num_pages_vbuffer * PAGE_SIZE;
	buf->vbuffer = vmalloc(buf->vbuffer_sz);
	if (!buf->vbuffer)
		return -ENOMEM;
	return 0;
}

static int xdrv_sh_buf_alloc(struct xenbus_device *xb_dev,
	struct xdrv_shared_buffer_info *buf,
	unsigned int buffer_size)
{
	int num_pages_vbuffer, num_grefs_per_page, num_pages_dir, num_grefs;
	int ret;

	xdrv_sh_buf_clear(buf);
	/* TODO: use XC_PAGE_SIZE */
	num_pages_vbuffer = DIV_ROUND_UP(buffer_size, PAGE_SIZE);
	/* number of grefs a page can hold with respect to the
	 * xendrm_page_directory header
	 */
	num_grefs_per_page = (PAGE_SIZE - sizeof(
		struct xendrm_page_directory)) / sizeof(grant_ref_t);
	/* number of pages the directory itself consumes */
	num_pages_dir = DIV_ROUND_UP(num_pages_vbuffer, num_grefs_per_page);
	num_grefs = num_pages_vbuffer + num_pages_dir;

	ret = xdrv_sh_buf_alloc_buffers(buf, num_pages_dir,
		num_pages_vbuffer, num_grefs);
	if (ret < 0)
		return ret;
	ret = xdrv_sh_buf_grant_refs(xb_dev, buf,
		num_pages_dir, num_pages_vbuffer, num_grefs);
	if (ret < 0)
		return ret;
	xdrv_sh_buf_fill_page_dir(buf, num_pages_dir);
	return 0;
}

static int xdrv_be_on_initwait(struct xdrv_info *drv_info)
{
	struct xenbus_device *xb_dev = drv_info->xb_dev;
	char **card_nodes;
	int stream_idx;
	int i, ret;

	card_nodes = xdrv_cfg_get_num_nodes(xb_dev->nodename,
		"connector", &drv_info->cfg_num_cards);
	kfree(card_nodes);
	if (!drv_info->cfg_num_cards) {
		dev_err(&drv_info->xb_dev->dev, "No sound cards configured");
		return 0;
	}
	drv_info->cfg_plat_data = devm_kzalloc(&drv_info->xb_dev->dev,
		drv_info->cfg_num_cards *
		sizeof(struct ddev_plat_data), GFP_KERNEL);
	if (!drv_info->cfg_plat_data)
		return -ENOMEM;
	/* stream index must be unique through all cards: pass it in to be
	 * incremented when creating streams
	 */
	stream_idx = 0;
	for (i = 0; i < drv_info->cfg_num_cards; i++) {
		/* read card configuration from the store and
		 * set platform data structure
		 */
		drv_info->cfg_plat_data[i].index = i;
		drv_info->cfg_plat_data[i].xdrv_info = drv_info;
#if 0
		ret = xdrv_cfg_card(drv_info,
			&drv_info->cfg_plat_data[i], &stream_idx);
		if (ret < 0)
			return ret;
#endif
	}
	/* create event channels for all streams and publish */
	return xdrv_evtchnl_create_all(drv_info, stream_idx);
}

static int xdrv_be_on_connected(struct xdrv_info *drv_info)
{
	return ddrv_init(drv_info);
}

static void xdrv_be_on_disconnected(struct xdrv_info *drv_info)
{
	xdrv_remove_internal(drv_info);
}

static void xdrv_be_on_changed(struct xenbus_device *xb_dev,
	enum xenbus_state backend_state)
{
	struct xdrv_info *drv_info = dev_get_drvdata(&xb_dev->dev);
	int ret;

	dev_dbg(&xb_dev->dev,
		"Backend state is %s, front is %s",
		xenbus_strstate(backend_state),
		xenbus_strstate(xb_dev->state));
	switch (backend_state) {
	case XenbusStateReconfiguring:
		/* fall through */
	case XenbusStateReconfigured:
		/* fall through */
	case XenbusStateInitialised:
		break;

	case XenbusStateInitialising:
		if (xb_dev->state == XenbusStateInitialising)
			break;
		/* recovering after backend unexpected closure */
		mutex_lock(&drv_info->mutex);
		xdrv_be_on_disconnected(drv_info);
		mutex_unlock(&drv_info->mutex);
		xenbus_switch_state(xb_dev, XenbusStateInitialising);
		break;

	case XenbusStateInitWait:
		if (xb_dev->state != XenbusStateInitialising)
			break;
		mutex_lock(&drv_info->mutex);
		ret = xdrv_be_on_initwait(drv_info);
		mutex_unlock(&drv_info->mutex);
		if (ret < 0) {
			xenbus_dev_fatal(xb_dev, ret, "initializing frontend");
			break;
		}
		xenbus_switch_state(xb_dev, XenbusStateInitialised);
		break;

	case XenbusStateConnected:
		if (xb_dev->state != XenbusStateInitialised)
			break;
		mutex_lock(&drv_info->mutex);
		ret = xdrv_be_on_connected(drv_info);
		mutex_unlock(&drv_info->mutex);
		if (ret < 0) {
			xenbus_dev_fatal(xb_dev, ret,
				"initializing DRM driver");
			break;
		}
		xenbus_switch_state(xb_dev, XenbusStateConnected);
		break;

	case XenbusStateUnknown:
		/* fall through */
	case XenbusStateClosed:
		if (xb_dev->state == XenbusStateClosed)
			break;
		if (xb_dev->state == XenbusStateInitialising)
			break;
		/* Missed the backend's CLOSING state -- fallthrough */
	case XenbusStateClosing:
		/* FIXME: is this check needed? */
		if (xb_dev->state == XenbusStateClosing)
			break;
		mutex_lock(&drv_info->mutex);
		xdrv_be_on_disconnected(drv_info);
		mutex_unlock(&drv_info->mutex);
		xenbus_switch_state(xb_dev, XenbusStateInitialising);
		break;
	}
}

static const struct xenbus_device_id xdrv_ids[] = {
	{ XENDRM_DRIVER_NAME },
	{ "" }
};

static struct xenbus_driver xen_driver = {
	.ids = xdrv_ids,
	.probe = xdrv_probe,
	.remove = xdrv_remove,
	.resume = xdrv_resume,
	.otherend_changed = xdrv_be_on_changed,
};

static int __init xdrv_init(void)
{
	if (!xen_domain())
		return -ENODEV;
	if (xen_initial_domain()) {
		pr_err(XENDRM_DRIVER_NAME " cannot run in Dom0\n");
		return -ENODEV;
	}
	if (!xen_has_pv_devices())
		return -ENODEV;
	pr_info("Registering XEN PV " XENDRM_DRIVER_NAME "\n");
	return xenbus_register_frontend(&xen_driver);
}

static void __exit xdrv_cleanup(void)
{
	pr_info("Unregistering XEN PV " XENDRM_DRIVER_NAME "\n");
	xenbus_unregister_driver(&xen_driver);
}

module_init(xdrv_init);
module_exit(xdrv_cleanup);

MODULE_DESCRIPTION("Xen virtual DRM device frontend");
MODULE_LICENSE("GPL");
MODULE_ALIAS("xen:"XENDRM_DRIVER_NAME);

