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

#include "xen-drm-timer.h"

static inline void xendrm_du_timer_restart(struct xendrm_du_timer *timer)
{
	mod_timer(&timer->timer, jiffies + timer->period);
}

void xendrm_du_timer_start(struct xendrm_du_timer *timer)
{
	unsigned long flags;

	spin_lock_irqsave(&timer->lock, flags);
	xendrm_du_timer_restart(timer);
	spin_unlock_irqrestore(&timer->lock, flags);
}

void xendrm_du_timer_restart_to(struct xendrm_du_timer *timer)
{
	atomic_set(&timer->to_cnt, timer->to_period);
}

void xendrm_du_timer_stop(struct xendrm_du_timer *timer)
{
	unsigned long flags;

	spin_lock_irqsave(&timer->lock, flags);
	del_timer_sync(&timer->timer);
	spin_unlock_irqrestore(&timer->lock, flags);
}

static void xendrm_du_timer_callback(unsigned long data)
{
	struct xendrm_du_timer *timer = (struct xendrm_du_timer *)data;
	unsigned long flags;

	spin_lock_irqsave(&timer->lock, flags);
	timer->clb->on_period(timer->clb_private);
	if (unlikely(atomic_dec_and_test(&timer->to_cnt)))
		timer->clb->on_timeout(timer->clb_private);
	xendrm_du_timer_restart(timer);
	spin_unlock_irqrestore(&timer->lock, flags);
}

int xendrm_du_timer_init(struct xendrm_du_timer *timer,
	unsigned long clb_private, struct xendrm_du_timer_callbacks *clb)
{
	if (!clb)
		return -EINVAL;
	timer->clb = clb;
	timer->clb_private = clb_private;
	setup_timer(&timer->timer, xendrm_du_timer_callback,
		(unsigned long)timer);
	spin_lock_init(&timer->lock);
	return 0;
}

void xendrm_du_timer_setup(struct xendrm_du_timer *timer,
	int freq_hz, int to_ms)
{
	timer->period = msecs_to_jiffies(1000 / freq_hz);
	timer->to_period = to_ms * freq_hz / 1000;
}
