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

#ifndef __XEN_DRM_TIMER_H_
#define __XEN_DRM_TIMER_H_

#include <linux/time.h>

struct xendrm_du_timer_callbacks {
	void (*on_period)(unsigned long data);
	void (*on_timeout)(unsigned long data);
};

struct xendrm_du_timer {
	struct timer_list timer;
	spinlock_t lock;
	unsigned long period;
	atomic_t to_cnt;
	int to_period;
	unsigned long clb_private;
	struct xendrm_du_timer_callbacks *clb;
};

int xendrm_du_timer_init(struct xendrm_du_timer *timer,
	unsigned long clb_private, struct xendrm_du_timer_callbacks *clb);
void xendrm_du_timer_setup(struct xendrm_du_timer *timer,
	int freq_hz, int to_ms);
void xendrm_du_timer_start(struct xendrm_du_timer *timer);
void xendrm_du_timer_stop(struct xendrm_du_timer *timer);
void xendrm_du_timer_restart_to(struct xendrm_du_timer *timer);

#endif /* __XEN_DRM_TIMER_H_ */
