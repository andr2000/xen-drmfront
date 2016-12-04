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

void xendrm_du_timer_start(struct xendrm_du_timer *timer)
{
	unsigned long flags;

	spin_lock_irqsave(&timer->lock, flags);
	atomic_set(&timer->running, 1);
#ifdef CONFIG_HIGH_RES_TIMERS
	hrtimer_start(&timer->timer, timer->period, HRTIMER_MODE_REL);
#else
	mod_timer(&timer->timer, jiffies + timer->period);
#endif
	spin_unlock_irqrestore(&timer->lock, flags);
}

void xendrm_du_timer_restart_to(struct xendrm_du_timer *timer)
{
	atomic_set(&timer->to_cnt, timer->to_period);
}

void xendrm_du_timer_stop(struct xendrm_du_timer *timer)
{
	unsigned long flags;

	atomic_set(&timer->running, 0);
	spin_lock_irqsave(&timer->lock, flags);
#ifdef CONFIG_HIGH_RES_TIMERS
	hrtimer_cancel(&timer->timer);
#else
	del_timer_sync(&timer->timer);
#endif
	/* this is the right place to kill tasklet, but DRM may call
	 * us from drm_irq.c:vblank->disable_timer
	 * it means that we might be in interrupt context now, so we cannot
	 * safely kill tasklet: postpone until cleanup
	 * running flag was set to 0, so tasklet won't reschedule
	 */
	spin_unlock_irqrestore(&timer->lock, flags);
}

static void xendrm_du_timer_handler(unsigned long data)
{
	struct xendrm_du_timer *timer = (struct xendrm_du_timer *)data;
	unsigned long flags;

	spin_lock_irqsave(&timer->lock, flags);
	timer->clb->on_period(timer->clb_private);
	if (unlikely(atomic_dec_and_test(&timer->to_cnt)))
		timer->clb->on_timeout(timer->clb_private);
	spin_unlock_irqrestore(&timer->lock, flags);
}

#ifdef CONFIG_HIGH_RES_TIMERS
static enum hrtimer_restart xendrm_du_timer_callback(struct hrtimer *hrtimer)
{
	struct xendrm_du_timer *timer =
		container_of(hrtimer, struct xendrm_du_timer, timer);

	if (likely(atomic_read(&timer->running))) {
		tasklet_schedule(&timer->tasklet);
		hrtimer_forward_now(hrtimer, timer->period);
		return HRTIMER_RESTART;
	}
	return HRTIMER_NORESTART;
}
#else
static void xendrm_du_timer_callback(unsigned long data)
{
	struct xendrm_du_timer *timer = (struct xendrm_du_timer *)data;

	if (likely(atomic_read(&timer->running))) {
		tasklet_schedule(&timer->tasklet);
		mod_timer(&timer->timer, jiffies + timer->period);
	}
}
#endif

int xendrm_du_timer_init(struct xendrm_du_timer *timer,
	unsigned long clb_private, struct xendrm_du_timer_callbacks *clb)
{
	if (!clb)
		return -EINVAL;
	timer->clb = clb;
	timer->clb_private = clb_private;

#ifdef CONFIG_HIGH_RES_TIMERS
	hrtimer_init(&timer->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	timer->timer.function = xendrm_du_timer_callback;
#else
	setup_timer(&timer->timer, xendrm_du_timer_callback,
		(unsigned long)timer);
#endif
	spin_lock_init(&timer->lock);
	tasklet_init(&timer->tasklet, xendrm_du_timer_handler,
		(unsigned long)timer);
	return 0;
}

void xendrm_du_timer_setup(struct xendrm_du_timer *timer,
	int freq_hz, int to_ms)
{
#ifdef CONFIG_HIGH_RES_TIMERS
	timer->period = ktime_set(1 / freq_hz, 1000000000UL / freq_hz);
#else
	timer->period = msecs_to_jiffies(1000 / freq_hz);
#endif
	timer->to_period = to_ms * freq_hz / 1000;
}

void xendrm_du_timer_cleanup(struct xendrm_du_timer *timer)
{
	unsigned long flags;

	spin_lock_irqsave(&timer->lock, flags);
	tasklet_kill(&timer->tasklet);
	spin_unlock_irqrestore(&timer->lock, flags);
}
