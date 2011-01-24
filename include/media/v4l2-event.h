/*
 * include/media/v4l2-event.h
 *
 * V4L2 events.
 *
 * Copyright (C) 2009 Nokia Corporation.
 *
 * Contact: Sakari Ailus <sakari.ailus at maxwell.research.nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#ifndef V4L2_EVENT_H
#define V4L2_EVENT_H

#include <linux/types.h>
#include <linux/videodev2.h>

#include <asm/atomic.h>

#define V4L2_MAX_EVENTS		1024 /* Ought to be enough for everyone. */

struct video_device;

struct _v4l2_event {
	struct list_head	list;
	struct v4l2_event	event;
};

struct v4l2_events {
	spinlock_t		lock; /* Protect everything here. */
	struct list_head	available;
	atomic_t		navailable;
	u32			sequence;
	wait_queue_head_t	wait;
	struct list_head	subscribed; /* Subscribed events. */
};

struct v4l2_subscribed_event {
	struct list_head	list;
	u32			type;
};

int v4l2_event_init(void);
void v4l2_event_exit(void);

void v4l2_event_init_fh(struct v4l2_events *events);
void v4l2_event_del(struct v4l2_events *events);

int v4l2_event_dequeue(struct v4l2_events *events, struct v4l2_event *event);

struct v4l2_subscribed_event *v4l2_event_subscribed(struct v4l2_events *sub,
						    u32 type);

void v4l2_event_queue(struct video_device *vdev, struct v4l2_event *ev);
int v4l2_event_pending(struct v4l2_events *events);

int v4l2_event_subscribe(struct v4l2_events *sub,
			 struct v4l2_event_subscription *s);
int v4l2_event_unsubscribe(struct v4l2_events *sub,
			   struct v4l2_event_subscription *s);

#endif /* V4L2_EVENT_H */
