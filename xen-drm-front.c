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

#include "displif.h"

#include "xen-drm.h"
#include "xen-drm-front.h"
#include "xen-drm-logs.h"

#ifndef XC_PAGE_SIZE
#define XC_PAGE_SIZE		PAGE_SIZE
#warning "XC_PAGE_SIZE is not defined, assuming PAGE_SIZE"
#endif

#define GRANT_INVALID_REF	0
/* timeout in ms to wait for backend to respond */
#define VDRM_WAIT_BACK_MS	5000

/* all operations which are not connector oriented use this ctrl event channel,
 * e.g. fb_attach/destroy which belong to a DRM device, not to a CRTC
 */
#define GENERIC_OP_EVT_CHNL	0

enum xdrv_evtchnl_state {
	EVTCHNL_STATE_DISCONNECTED,
	EVTCHNL_STATE_CONNECTED,
	EVTCHNL_STATE_SUSPENDED,
};

enum xdrv_evtchnl_type {
	EVTCHNL_TYPE_CTRL,
	EVTCHNL_TYPE_EVT,
};

struct xdrv_evtchnl_info {
	struct xdrv_info *drv_info;
	int gref;
	int port;
	int irq;
	/* state of the event channel */
	enum xdrv_evtchnl_state state;
	enum xdrv_evtchnl_type type;
	union {
		struct {
			struct xen_displif_front_ring ring;
			struct completion completion;
			/* latest response status and id */
			int resp_status;
			uint16_t resp_id;
			uint16_t req_next_id;
		} ctrl;
		struct {
			struct xendispl_event_page *page;
		} evt;
	} u;
};

struct xdrv_shared_buffer_info {
	struct list_head list;
	uint64_t dumb_cookie;
	int num_grefs;
	grant_ref_t *grefs;
	unsigned char *vdirectory;
	unsigned char *vbuffer;
	size_t vbuffer_sz;
};

struct xdrv_evtchnl_pair_info {
	struct xdrv_evtchnl_info ctrl;
	struct xdrv_evtchnl_info evt;
};

struct xdrv_info {
	struct xenbus_device *xb_dev;
	spinlock_t io_lock;
	struct mutex io_generic_evt_lock;
	struct mutex mutex;
	bool ddrv_registered;
	/* virtual DRM platform device */
	struct platform_device *ddrv_pdev;

	int num_evt_pairs;
	struct xdrv_evtchnl_pair_info *evt_pairs;
	struct xendrm_plat_data cfg_plat_data;

	/* dumb buffers */
	struct xdrv_shared_buffer_info *dumb_buf;
};

struct DRMIF_TO_KERN_ERROR {
	int drmif;
	int kern;
};

static struct DRMIF_TO_KERN_ERROR drmif_kern_error_codes[] = {
	{ .drmif = XENDISPL_RSP_OKAY,     .kern = 0 },
	{ .drmif = XENDISPL_RSP_ERROR,    .kern = EIO },
};

static int drmif_to_kern_error(int drmif_err)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(drmif_kern_error_codes); i++)
		if (drmif_kern_error_codes[i].drmif == drmif_err)
			return -drmif_kern_error_codes[i].kern;
	return -EIO;
}

static inline void xdrv_evtchnl_flush(
		struct xdrv_evtchnl_info *channel);
static struct xdrv_shared_buffer_info *xdrv_sh_buf_alloc(
	struct xdrv_info *drv_info, uint64_t dumb_cookie,
	void *vbuffer, unsigned int buffer_size);
static grant_ref_t xdrv_sh_buf_get_dir_start(
	struct xdrv_shared_buffer_info *buf);
static void xdrv_sh_buf_free_by_cookie(struct xdrv_info *drv_info,
	uint64_t dumb_cookie);

static inline struct xendispl_req *ddrv_be_prepare_req(
	struct xdrv_evtchnl_info *evtchnl, uint8_t operation)
{
	struct xendispl_req *req;

	req = RING_GET_REQUEST(&evtchnl->u.ctrl.ring,
		evtchnl->u.ctrl.ring.req_prod_pvt);
	req->operation = operation;
	req->id = evtchnl->u.ctrl.req_next_id++;
	evtchnl->u.ctrl.resp_id = req->id;
	return req;
}

/* CAUTION!!! Call this with the spin lock held.
 * This function will release it
 */
static int ddrv_be_stream_do_io(struct xdrv_evtchnl_info *evtchnl,
	struct xendispl_req *req, unsigned long flags)
{
	int ret;

	reinit_completion(&evtchnl->u.ctrl.completion);
	if (unlikely(evtchnl->state != EVTCHNL_STATE_CONNECTED)) {
		spin_unlock_irqrestore(&evtchnl->drv_info->io_lock, flags);
		return -EIO;
	}
	xdrv_evtchnl_flush(evtchnl);
	spin_unlock_irqrestore(&evtchnl->drv_info->io_lock, flags);
	ret = 0;
	if (wait_for_completion_interruptible_timeout(
			&evtchnl->u.ctrl.completion,
			msecs_to_jiffies(VDRM_WAIT_BACK_MS)) <= 0)
		ret = -ETIMEDOUT;
	if (ret < 0)
		return ret;
	return drmif_to_kern_error(evtchnl->u.ctrl.resp_status);
}

int xendispl_front_mode_set(struct xendrm_du_crtc *du_crtc, uint32_t x,
	uint32_t y, uint32_t width, uint32_t height, uint32_t bpp,
	uint64_t fb_cookie)

{
	struct xdrv_evtchnl_info *evtchnl;
	struct xdrv_info *drv_info;
	struct xendispl_req *req;
	unsigned long flags;
	int ret;

	drv_info = du_crtc->xendrm_du->xdrv_info;
	evtchnl = &drv_info->evt_pairs[du_crtc->index].ctrl;
	if (unlikely(!evtchnl))
		return -EIO;
	mutex_lock(&drv_info->io_generic_evt_lock);
	spin_lock_irqsave(&drv_info->io_lock, flags);
	req = ddrv_be_prepare_req(evtchnl, XENDISPL_OP_SET_CONFIG);
	req->op.set_config.x = x;
	req->op.set_config.y = y;
	req->op.set_config.width = width;
	req->op.set_config.height = height;
	req->op.set_config.bpp = bpp;
	req->op.set_config.fb_cookie = fb_cookie;
	ret = ddrv_be_stream_do_io(evtchnl, req, flags);
	mutex_unlock(&drv_info->io_generic_evt_lock);
	return ret;
}

int xendispl_front_dbuf_create(struct xdrv_info *drv_info, uint64_t dumb_cookie,
	uint32_t width, uint32_t height, uint32_t bpp, uint64_t size,
	void *vaddr)
{
	struct xdrv_evtchnl_info *evtchnl;
	struct xdrv_shared_buffer_info *buf;
	struct xendispl_req *req;
	unsigned long flags;
	int ret;

	evtchnl = &drv_info->evt_pairs[GENERIC_OP_EVT_CHNL].ctrl;
	if (unlikely(!evtchnl))
		return -EIO;
	mutex_lock(&drv_info->io_generic_evt_lock);
	buf = xdrv_sh_buf_alloc(drv_info, dumb_cookie, vaddr, size);
	if (!buf) {
		mutex_unlock(&drv_info->io_generic_evt_lock);
		return -ENOMEM;
	}
	spin_lock_irqsave(&drv_info->io_lock, flags);
	req = ddrv_be_prepare_req(evtchnl, XENDISPL_OP_DBUF_CREATE);
	req->op.dbuf_create.gref_directory_start =
		xdrv_sh_buf_get_dir_start(buf);
	req->op.dbuf_create.buffer_sz = size;
	req->op.dbuf_create.dbuf_cookie = dumb_cookie;
	req->op.dbuf_create.width = width;
	req->op.dbuf_create.height = height;
	req->op.dbuf_create.bpp = bpp;
	ret = ddrv_be_stream_do_io(evtchnl, req, flags);
	mutex_unlock(&drv_info->io_generic_evt_lock);
	return ret;
}

int xendispl_front_dbuf_destroy(struct xdrv_info *drv_info, uint64_t dumb_cookie)
{
	struct xdrv_evtchnl_info *evtchnl;
	struct xendispl_req *req;
	unsigned long flags;
	int ret;

	mutex_lock(&drv_info->io_generic_evt_lock);
	evtchnl = &drv_info->evt_pairs[GENERIC_OP_EVT_CHNL].ctrl;
	if (unlikely(!evtchnl)) {
		mutex_unlock(&drv_info->io_generic_evt_lock);
		return -EIO;
	}
	spin_lock_irqsave(&drv_info->io_lock, flags);
	req = ddrv_be_prepare_req(evtchnl, XENDISPL_OP_DBUF_DESTROY);

	req->op.dbuf_destroy.dbuf_cookie = dumb_cookie;
	ret = ddrv_be_stream_do_io(evtchnl, req, flags);
	xdrv_sh_buf_free_by_cookie(drv_info, dumb_cookie);
	mutex_unlock(&drv_info->io_generic_evt_lock);
	return ret;
}

int xendispl_front_fb_attach(struct xdrv_info *drv_info,
	uint64_t dumb_cookie, uint64_t fb_cookie, uint32_t width,
	uint32_t height, uint32_t pixel_format)
{
	struct xdrv_evtchnl_info *evtchnl;
	struct xendispl_req *req;
	unsigned long flags;
	int ret;

	evtchnl = &drv_info->evt_pairs[GENERIC_OP_EVT_CHNL].ctrl;
	if (unlikely(!evtchnl))
		return -EIO;
	mutex_lock(&drv_info->io_generic_evt_lock);
	spin_lock_irqsave(&drv_info->io_lock, flags);
	req = ddrv_be_prepare_req(evtchnl, XENDISPL_OP_FB_ATTACH);
	req->op.fb_attach.dbuf_cookie = dumb_cookie;
	req->op.fb_attach.fb_cookie = fb_cookie;
	req->op.fb_attach.width = width;
	req->op.fb_attach.height = height;
	req->op.fb_attach.pixel_format = pixel_format;
	ret = ddrv_be_stream_do_io(evtchnl, req, flags);
	mutex_unlock(&drv_info->io_generic_evt_lock);
	return ret;
}

int xendispl_front_fb_detach(struct xdrv_info *drv_info, uint64_t fb_cookie)
{
	struct xdrv_evtchnl_info *evtchnl;
	struct xendispl_req *req;
	unsigned long flags;
	int ret;

	evtchnl = &drv_info->evt_pairs[GENERIC_OP_EVT_CHNL].ctrl;
	if (unlikely(!evtchnl))
		return -EIO;
	mutex_lock(&drv_info->io_generic_evt_lock);
	spin_lock_irqsave(&drv_info->io_lock, flags);
	req = ddrv_be_prepare_req(evtchnl, XENDISPL_OP_FB_DETACH);
	req->op.fb_detach.fb_cookie = fb_cookie;
	ret = ddrv_be_stream_do_io(evtchnl, req, flags);
	mutex_unlock(&drv_info->io_generic_evt_lock);
	return ret;
}

int xendispl_front_page_flip(struct xdrv_info *drv_info, int crtc_idx,
	uint64_t fb_cookie)
{
	struct xdrv_evtchnl_info *evtchnl;
	struct xendispl_req *req;
	unsigned long flags;
	int ret;

	if (unlikely(crtc_idx >= drv_info->num_evt_pairs))
		return -EINVAL;
	evtchnl = &drv_info->evt_pairs[crtc_idx].ctrl;
	mutex_lock(&drv_info->io_generic_evt_lock);
	spin_lock_irqsave(&drv_info->io_lock, flags);
	req = ddrv_be_prepare_req(evtchnl, XENDISPL_OP_PG_FLIP);
	req->op.pg_flip.conn_idx = crtc_idx;
	req->op.pg_flip.fb_cookie = fb_cookie;
	ret = ddrv_be_stream_do_io(evtchnl, req, flags);
	mutex_unlock(&drv_info->io_generic_evt_lock);
	return ret;
}

static struct xendispl_front_funcs xendispl_front_funcs = {
	.mode_set = xendispl_front_mode_set,
	.dbuf_create = xendispl_front_dbuf_create,
	.dbuf_destroy = xendispl_front_dbuf_destroy,
	.fb_attach = xendispl_front_fb_attach,
	.fb_detach = xendispl_front_fb_detach,
	.page_flip = xendispl_front_page_flip,
};

static int ddrv_probe(struct platform_device *pdev)
{
	return xendrm_probe(pdev, &xendispl_front_funcs);
}

static int ddrv_remove(struct platform_device *pdev)
{
	return xendrm_remove(pdev);
}

struct platform_device_info ddrv_platform_info = {
	.name = XENDISPL_DRIVER_NAME,
	.id = 0,
	.num_res = 0,
	.dma_mask = DMA_BIT_MASK(32),
};

static struct platform_driver ddrv_info = {
	.probe		= ddrv_probe,
	.remove		= ddrv_remove,
	.driver		= {
		.name	= XENDISPL_DRIVER_NAME,
	},
};

static void ddrv_cleanup(struct xdrv_info *drv_info)
{
	if (!drv_info->ddrv_registered)
		return;
	if (drv_info->ddrv_pdev)
		platform_device_unregister(drv_info->ddrv_pdev);
	platform_driver_unregister(&ddrv_info);
	drv_info->ddrv_registered = false;
	drv_info->ddrv_pdev = NULL;
}

static int ddrv_init(struct xdrv_info *drv_info)
{
	struct xendrm_plat_data *platdata;
	int ret;

	ret = platform_driver_register(&ddrv_info);
	if (ret < 0)
		return ret;
	drv_info->ddrv_registered = true;
	platdata = &drv_info->cfg_plat_data;
	/* pass card configuration via platform data */
	ddrv_platform_info.data = platdata;
	ddrv_platform_info.size_data = sizeof(struct xendrm_plat_data);
	drv_info->ddrv_pdev = platform_device_register_full(&ddrv_platform_info);
	if (IS_ERR(drv_info->ddrv_pdev)) {
		drv_info->ddrv_pdev = NULL;
		goto fail;
	}
	return 0;

fail:
	dev_err(&drv_info->xb_dev->dev, "Failed to register DRM driver");
	ddrv_cleanup(drv_info);
	return -ENODEV;
}

static irqreturn_t xdrv_evtchnl_interrupt_ctrl(int irq, void *dev_id)
{
	struct xdrv_evtchnl_info *channel = dev_id;
	struct xdrv_info *drv_info = channel->drv_info;
	struct xendispl_resp *resp;
	RING_IDX i, rp;
	unsigned long flags;

	spin_lock_irqsave(&drv_info->io_lock, flags);
	if (unlikely(channel->state != EVTCHNL_STATE_CONNECTED))
		goto out;

 again:
	rp = channel->u.ctrl.ring.sring->rsp_prod;
	rmb(); /* Ensure we see queued responses up to 'rp'. */

	for (i = channel->u.ctrl.ring.rsp_cons; i != rp; i++) {
		resp = RING_GET_RESPONSE(&channel->u.ctrl.ring, i);
		if (resp->id != channel->u.ctrl.resp_id)
			continue;
		switch (resp->operation) {
		case XENDISPL_OP_PG_FLIP:
		case XENDISPL_OP_FB_ATTACH:
		case XENDISPL_OP_FB_DETACH:
		case XENDISPL_OP_DBUF_CREATE:
		case XENDISPL_OP_DBUF_DESTROY:
		case XENDISPL_OP_SET_CONFIG:
			channel->u.ctrl.resp_status = resp->status;
			complete(&channel->u.ctrl.completion);
			break;
		default:
			dev_err(&drv_info->xb_dev->dev,
				"Operation %d is not supported",
				resp->operation);
			break;
		}
	}

	channel->u.ctrl.ring.rsp_cons = i;

	if (i != channel->u.ctrl.ring.req_prod_pvt) {
		int more_to_do;

		RING_FINAL_CHECK_FOR_RESPONSES(&channel->u.ctrl.ring,
			more_to_do);
		if (more_to_do)
			goto again;
	} else
		channel->u.ctrl.ring.sring->rsp_event = i + 1;

out:
	spin_unlock_irqrestore(&drv_info->io_lock, flags);
	return IRQ_HANDLED;
}

static irqreturn_t xdrv_evtchnl_interrupt_evt(int irq, void *dev_id)
{
	struct xdrv_evtchnl_info *channel = dev_id;
	struct xdrv_info *drv_info = channel->drv_info;
	struct xendispl_event_page *page = channel->u.evt.page;
	uint32_t cons, prod;
	unsigned long flags;

	spin_lock_irqsave(&drv_info->io_lock, flags);
//	LOG0("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx got event, state %d", channel->state);
	if (unlikely(channel->state != EVTCHNL_STATE_CONNECTED))
		goto out;

	prod = page->in_prod;
	if (prod == page->in_cons)
		goto out;
	/* ensure we see ring contents up to prod */
	rmb();
	for (cons = page->in_cons; cons != prod; cons++) {
		struct xendispl_evt *event;

		event = &XENDISPL_IN_RING_REF(page, cons);
//		LOG0("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx got event, type %d", event->u.data.type);
		switch (event->type) {
		case XENDISPL_EVT_PG_FLIP:
			if (likely(xendispl_front_funcs.on_page_flip)) {
				xendispl_front_funcs.on_page_flip(
					drv_info->ddrv_pdev,
					event->op.pg_flip.conn_idx,
					event->op.pg_flip.fb_cookie);
			}
			break;
		}
	}
	/* ensure we got ring contents */
	mb();
	page->in_cons = cons;
	notify_remote_via_irq(channel->irq);

out:
	spin_unlock_irqrestore(&drv_info->io_lock, flags);
	return IRQ_HANDLED;
}

static void xdrv_evtchnl_free(struct xdrv_info *drv_info,
		struct xdrv_evtchnl_info *channel)
{
	unsigned long page = 0;

	if (channel->type == EVTCHNL_TYPE_CTRL)
		page = (unsigned long)channel->u.ctrl.ring.sring;
	else if (channel->type == EVTCHNL_TYPE_EVT)
		page = (unsigned long)channel->u.evt.page;
	if (!page)
		return;
	channel->state = EVTCHNL_STATE_DISCONNECTED;
	if (channel->type == EVTCHNL_TYPE_CTRL) {
		/* release all who still waits for response if any */
		channel->u.ctrl.resp_status = -XENDISPL_RSP_ERROR;
		complete_all(&channel->u.ctrl.completion);
	}
	if (channel->irq)
		unbind_from_irqhandler(channel->irq, channel);
	channel->irq = 0;
	if (channel->port)
		xenbus_free_evtchn(drv_info->xb_dev, channel->port);
	channel->port = 0;
	/* End access and free the pages */
	if (channel->gref != GRANT_INVALID_REF)
		gnttab_end_foreign_access(channel->gref, 0, page);
	channel->gref = GRANT_INVALID_REF;
	if (channel->type == EVTCHNL_TYPE_CTRL)
		channel->u.ctrl.ring.sring = NULL;
	else
		channel->u.evt.page = NULL;
}

static void xdrv_evtchnl_free_all(struct xdrv_info *drv_info)
{
	int i;

	if (!drv_info->evt_pairs)
		return;
	for (i = 0; i < drv_info->num_evt_pairs; i++) {
		xdrv_evtchnl_free(drv_info,
			&drv_info->evt_pairs[i].ctrl);
		xdrv_evtchnl_free(drv_info,
			&drv_info->evt_pairs[i].evt);
	}
	devm_kfree(&drv_info->xb_dev->dev, drv_info->evt_pairs);
	drv_info->evt_pairs = NULL;
}

static int xdrv_evtchnl_alloc(struct xdrv_info *drv_info,
	struct xdrv_evtchnl_info *evt_channel,
	enum xdrv_evtchnl_type type)
{
	struct xenbus_device *xb_dev = drv_info->xb_dev;
	unsigned long page;
	grant_ref_t gref;
	irq_handler_t handler;
	int ret;

	evt_channel->type = type;
	evt_channel->drv_info = drv_info;
	evt_channel->state = EVTCHNL_STATE_DISCONNECTED;
	evt_channel->gref = GRANT_INVALID_REF;
	evt_channel->port = 0;
	evt_channel->irq = 0;
	memset(&evt_channel->u, 0, sizeof(evt_channel->u));
	page = get_zeroed_page(GFP_NOIO | __GFP_HIGH);
	if (!page) {
		ret = -ENOMEM;
		goto fail;
	}

	if (type == EVTCHNL_TYPE_CTRL) {
		struct xen_displif_sring *sring;

		init_completion(&evt_channel->u.ctrl.completion);
		sring = (struct xen_displif_sring *)page;
		SHARED_RING_INIT(sring);
		/* TODO: use XC_PAGE_SIZE */
		FRONT_RING_INIT(&evt_channel->u.ctrl.ring, sring, XC_PAGE_SIZE);

		ret = xenbus_grant_ring(xb_dev, sring, 1, &gref);
		if (ret < 0)
			goto fail;
		handler = xdrv_evtchnl_interrupt_ctrl;
	} else {
		evt_channel->u.evt.page = (struct xendispl_event_page *)page;
		ret = gnttab_grant_foreign_access(xb_dev->otherend_id,
			virt_to_gfn(page), 0);
		if (ret < 0)
			goto fail;
		gref = ret;
		handler = xdrv_evtchnl_interrupt_evt;
	}
	evt_channel->gref = gref;

	ret = xenbus_alloc_evtchn(xb_dev, &evt_channel->port);
	if (ret < 0)
		goto fail;

	ret = bind_evtchn_to_irqhandler(evt_channel->port,
		handler, 0, xb_dev->devicetype, evt_channel);
	if (ret < 0)
		goto fail;
	evt_channel->irq = ret;
	return 0;

fail:
	dev_err(&xb_dev->dev, "Failed to allocate ring with err %d", ret);
	return ret;
}

static int xdrv_evtchnl_create(struct xdrv_info *drv_info,
	struct xdrv_evtchnl_info *evt_channel, enum xdrv_evtchnl_type type,
	const char *path, const char *node_ring,
	const char *node_chnl)
{
	const char *message;
	int ret;

	/* allocate and open control channel */
	ret = xdrv_evtchnl_alloc(drv_info, evt_channel, type);
	if (ret < 0) {
		message = "allocating event channel";
		goto fail;
	}
	/* Write control channel ring reference */
	ret = xenbus_printf(XBT_NIL, path, node_ring, "%u",
			evt_channel->gref);
	if (ret < 0) {
		message = "writing ring-ref";
		goto fail;
	}

	ret = xenbus_printf(XBT_NIL, path, node_chnl, "%u",
		evt_channel->port);
	if (ret < 0) {
		message = "writing event channel";
		goto fail;
	}
	return 0;

fail:
	LOG0("Error %s with err %d", message, ret);
	return ret;
}

static inline void xdrv_evtchnl_flush(
		struct xdrv_evtchnl_info *channel)
{
	int notify;

	channel->u.ctrl.ring.req_prod_pvt++;
	RING_PUSH_REQUESTS_AND_CHECK_NOTIFY(&channel->u.ctrl.ring, notify);
	if (notify)
		notify_remote_via_irq(channel->irq);
}

static int xdrv_evtchnl_create_all(struct xdrv_info *drv_info)
{
	struct xendrm_plat_data *plat_data;
	int ret, conn;

	plat_data = &drv_info->cfg_plat_data;
	drv_info->evt_pairs = devm_kcalloc(&drv_info->xb_dev->dev,
		plat_data->num_connectors,
		sizeof(struct xdrv_evtchnl_pair_info), GFP_KERNEL);
	if (!drv_info->evt_pairs) {
		ret = -ENOMEM;
		goto fail;
	}
	for (conn = 0; conn < plat_data->num_connectors; conn++) {
		ret = xdrv_evtchnl_create(drv_info,
			&drv_info->evt_pairs[conn].ctrl,
			EVTCHNL_TYPE_CTRL,
			plat_data->connectors[conn].xenstore_path,
			XENDISPL_FIELD_CTRL_RING_REF,
			XENDISPL_FIELD_CTRL_CHANNEL);
		if (ret < 0)
			goto fail;
		ret = xdrv_evtchnl_create(drv_info,
			&drv_info->evt_pairs[conn].evt,
			EVTCHNL_TYPE_EVT,
			plat_data->connectors[conn].xenstore_path,
			XENDISPL_FIELD_EVT_RING_REF,
			XENDISPL_FIELD_EVT_CHANNEL);
		if (ret < 0)
			goto fail;
	}
	drv_info->num_evt_pairs = plat_data->num_connectors;
	return 0;
fail:
	xdrv_evtchnl_free_all(drv_info);
	return ret;
}

static void xdrv_evtchnl_set_state(struct xdrv_info *drv_info,
	enum xdrv_evtchnl_state state)
{
	unsigned long flags;
	int i;

	if (!drv_info->evt_pairs)
		return;
	spin_lock_irqsave(&drv_info->io_lock, flags);
	for (i = 0; i < drv_info->num_evt_pairs; i++) {
		drv_info->evt_pairs[i].ctrl.state = state;
		drv_info->evt_pairs[i].evt.state = state;
	}
	spin_unlock_irqrestore(&drv_info->io_lock, flags);

}
/* get number of nodes under the path to get number of
 * cards configured or number of connectors within the card
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

static int xdrv_cfg_connector(struct xdrv_info *drv_info,
	struct xendrm_cfg_connector *connector,
	const char *path, int index)
{
	char *connector_path;
	int ret;

	connector_path = devm_kasprintf(&drv_info->xb_dev->dev,
		GFP_KERNEL, "%s/%s/%d", path, XENDISPL_PATH_CONNECTOR, index);
	if (!connector_path)
		return -ENOMEM;
	connector->xenstore_path = connector_path;
	if (xenbus_scanf(XBT_NIL, connector_path, XENDISPL_FIELD_RESOLUTION,
			"%d" XENDISPL_RESOLUTION_SEPARATOR "%d",
			&connector->width, &connector->height) < 0) {
		connector->width = 0;
		connector->height = 0;
		ret = -EINVAL;
		LOG0("Wrong connector resolution");
		goto fail;
	}
	LOG0("Connector %s: resolution %dx%d",
		connector_path, connector->width, connector->height);
	ret = 0;
fail:
	return -ret;
}

static int xdrv_cfg_card(struct xdrv_info *drv_info,
	struct xendrm_plat_data *plat_data)
{
	struct xenbus_device *xb_dev = drv_info->xb_dev;
	const char *path;
	char **connector_nodes = NULL;
	int ret, num_conn, i;

	path = xb_dev->nodename;
	plat_data->num_connectors = 0;
	connector_nodes = xdrv_cfg_get_num_nodes(path, XENDISPL_PATH_CONNECTOR,
		&num_conn);
	kfree(connector_nodes);
	if (!num_conn) {
		LOG0("No connectors configured at %s/%s",
			path, XENDISPL_PATH_CONNECTOR);
		return -ENODEV;
	}
	if (num_conn > XENDRM_DU_MAX_CRTCS) {
		LOG0("Only %d connectors supported, skipping the rest",
			XENDRM_DU_MAX_CRTCS);
		num_conn = XENDRM_DU_MAX_CRTCS;
	}
	for (i = 0; i < num_conn; i++) {
		ret = xdrv_cfg_connector(drv_info,
			&plat_data->connectors[i], path, i);
		if (ret < 0)
			return ret;
	}
	plat_data->num_connectors = num_conn;
	return 0;
}

static grant_ref_t xdrv_sh_buf_get_dir_start(
	struct xdrv_shared_buffer_info *buf)
{
	if (!buf->grefs)
		return GRANT_INVALID_REF;
	return buf->grefs[0];
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
	kfree(buf);
}

static void xdrv_sh_buf_free_by_cookie(struct xdrv_info *drv_info,
	uint64_t dumb_cookie)
{
	struct list_head *pos, *q;

	list_for_each_safe(pos, q, &drv_info->dumb_buf->list) {
		struct xdrv_shared_buffer_info *buf;

		buf = list_entry(pos, struct xdrv_shared_buffer_info, list);
		if (buf->dumb_cookie == dumb_cookie) {
			list_del(pos);
			xdrv_sh_buf_free(buf);
			if (drv_info->dumb_buf == buf)
				drv_info->dumb_buf = NULL;
			break;
		}
	}
}

static void xdrv_sh_buf_free_all(struct xdrv_info *drv_info)
{
	struct list_head *pos, *q;

	mutex_lock(&drv_info->io_generic_evt_lock);
	if (!drv_info->dumb_buf)
		goto out;
	list_for_each_safe(pos, q, &drv_info->dumb_buf->list) {
		struct xdrv_shared_buffer_info *buf;

		buf = list_entry(pos, struct xdrv_shared_buffer_info, list);
		list_del(pos);
		xdrv_sh_buf_free(buf);
		if (drv_info->dumb_buf == buf)
			drv_info->dumb_buf = NULL;
	}
out:
	mutex_unlock(&drv_info->io_generic_evt_lock);
}

void xdrv_sh_buf_fill_page_dir(struct xdrv_shared_buffer_info *buf,
		int num_pages_dir)
{
	struct xendispl_page_directory *page_dir;
	unsigned char *ptr;
	int i, cur_gref, grefs_left, num_grefs_per_page, to_copy;

	ptr = buf->vdirectory;
	grefs_left = buf->num_grefs - num_pages_dir;
	num_grefs_per_page = (XC_PAGE_SIZE -
		offsetof(struct xendispl_page_directory, gref)) /
		sizeof(grant_ref_t);
	/* skip grefs at start, they are for pages granted for the directory */
	cur_gref = num_pages_dir;
	for (i = 0; i < num_pages_dir; i++) {
		page_dir = (struct xendispl_page_directory *)ptr;
		if (grefs_left <= num_grefs_per_page) {
			to_copy = grefs_left;
			page_dir->gref_dir_next_page = GRANT_INVALID_REF;
		} else {
			to_copy = num_grefs_per_page;
			page_dir->gref_dir_next_page = buf->grefs[i + 1];
		}
		memcpy(&page_dir->gref, &buf->grefs[cur_gref],
			to_copy * sizeof(grant_ref_t));
		ptr += XC_PAGE_SIZE;
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
				XC_PAGE_SIZE * i)), 0);
		buf->grefs[j++] = cur_ref;
	}
	for (i = 0; i < num_pages_vbuffer; i++) {
		cur_ref = gnttab_claim_grant_reference(&priv_gref_head);
		if (cur_ref < 0)
			return cur_ref;
		gnttab_grant_foreign_access_ref(cur_ref, otherend_id,
			xen_page_to_gfn(vmalloc_to_page(buf->vbuffer +
				XC_PAGE_SIZE * i)), 0);
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
	buf->vdirectory = vmalloc(num_pages_dir * XC_PAGE_SIZE);
	if (!buf->vdirectory)
		return -ENOMEM;
	buf->vbuffer_sz = num_pages_vbuffer * XC_PAGE_SIZE;
	return 0;
}

static struct xdrv_shared_buffer_info *
xdrv_sh_buf_alloc(struct xdrv_info *drv_info, uint64_t dumb_cookie,
	void *vbuffer, unsigned int buffer_size)
{
	struct xdrv_shared_buffer_info *buf;
	int num_pages_vbuffer, num_grefs_per_page, num_pages_dir, num_grefs;

	if (!vbuffer)
		return NULL;
	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return NULL;
	if (!drv_info->dumb_buf) {
		drv_info->dumb_buf = buf;
		INIT_LIST_HEAD(&buf->list);
	}
	buf->vbuffer = vbuffer;
	buf->dumb_cookie = dumb_cookie;
	/* TODO: use XC_PAGE_SIZE */
	num_pages_vbuffer = DIV_ROUND_UP(buffer_size, XC_PAGE_SIZE);
	/* number of grefs a page can hold with respect to the
	 * xendispl_page_directory header
	 */
	num_grefs_per_page = (XC_PAGE_SIZE - sizeof(
		struct xendispl_page_directory)) / sizeof(grant_ref_t);
	/* number of pages the directory itself consumes */
	num_pages_dir = DIV_ROUND_UP(num_pages_vbuffer, num_grefs_per_page);
	num_grefs = num_pages_vbuffer + num_pages_dir;

	if (xdrv_sh_buf_alloc_buffers(buf, num_pages_dir,
			num_pages_vbuffer, num_grefs) < 0)
		goto fail;
	if (xdrv_sh_buf_grant_refs(drv_info->xb_dev, buf,
			num_pages_dir, num_pages_vbuffer, num_grefs) < 0)
		goto fail;
	xdrv_sh_buf_fill_page_dir(buf, num_pages_dir);
	list_add(&(buf->list), &(drv_info->dumb_buf->list));
	return buf;
fail:
	if (drv_info->dumb_buf == buf)
		drv_info->dumb_buf = NULL;
	xdrv_sh_buf_free(buf);
	return NULL;
}

static void xdrv_remove_internal(struct xdrv_info *drv_info)
{
	ddrv_cleanup(drv_info);
	xdrv_evtchnl_free_all(drv_info);
	xdrv_sh_buf_free_all(drv_info);
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
	mutex_init(&drv_info->io_generic_evt_lock);
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

static int xdrv_be_on_initwait(struct xdrv_info *drv_info)
{
	struct xendrm_plat_data *cfg_plat_data;
	int ret;

	cfg_plat_data = &drv_info->cfg_plat_data;
	cfg_plat_data->xdrv_info = drv_info;
	ret = xdrv_cfg_card(drv_info, cfg_plat_data);
	if (ret < 0)
		return ret;
	LOG0("Have %d conectors", cfg_plat_data->num_connectors);
	/* create event channels for all streams and publish */
	return xdrv_evtchnl_create_all(drv_info);
}

static int xdrv_be_on_connected(struct xdrv_info *drv_info)
{
	xdrv_evtchnl_set_state(drv_info, EVTCHNL_STATE_CONNECTED);
	return ddrv_init(drv_info);
}

static void xdrv_be_on_disconnected(struct xdrv_info *drv_info)
{
	xdrv_remove_internal(drv_info);
	xdrv_evtchnl_set_state(drv_info, EVTCHNL_STATE_DISCONNECTED);
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
	{ XENDISPL_DRIVER_NAME },
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
		LOG0(XENDISPL_DRIVER_NAME " cannot run in Dom0");
		return -ENODEV;
	}
	if (!xen_has_pv_devices())
		return -ENODEV;
	LOG0("Registering XEN PV " XENDISPL_DRIVER_NAME);
	return xenbus_register_frontend(&xen_driver);
}

static void __exit xdrv_cleanup(void)
{
	LOG0("Unregistering XEN PV " XENDISPL_DRIVER_NAME);
	xenbus_unregister_driver(&xen_driver);
}

module_init(xdrv_init);
module_exit(xdrv_cleanup);

MODULE_DESCRIPTION("Xen virtual DRM device frontend");
MODULE_LICENSE("GPL");
MODULE_ALIAS("xen:"XENDISPL_DRIVER_NAME);

