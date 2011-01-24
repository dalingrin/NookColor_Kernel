/*
 * drivers/media/video/v4l2-event.c
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

#include <media/v4l2-dev.h>
#include <media/v4l2-event.h>

#include <linux/sched.h>

static struct kmem_cache *event_kmem;

int v4l2_event_init(void)
{
	event_kmem = kmem_cache_create("event_kmem",
				       sizeof(struct _v4l2_event), 0,
				       SLAB_HWCACHE_ALIGN,
				       NULL);

	if (!event_kmem)
		return -ENOMEM;

	return 0;
}

void v4l2_event_exit(void)
{
	if (!event_kmem)
		kmem_cache_destroy(event_kmem);

	event_kmem = NULL;
}

void v4l2_event_init_fh(struct v4l2_events *events)
{
	init_waitqueue_head(&events->wait);
	spin_lock_init(&events->lock);

	INIT_LIST_HEAD(&events->available);
	INIT_LIST_HEAD(&events->subscribed);

	atomic_set(&events->navailable, 0);
	events->sequence = 0;
};

void v4l2_event_del(struct v4l2_events *events)
{
	while (!list_empty(&events->available)) {
		struct _v4l2_event *ev;

		ev = list_entry(events->available.next,
				struct _v4l2_event, list);

		list_del(&ev->list);

		kmem_cache_free(event_kmem, ev);
	}

	while (!list_empty(&events->subscribed)) {
		struct v4l2_subscribed_event *sub;

		sub = list_entry(events->subscribed.next,
				struct v4l2_subscribed_event, list);

		list_del(&sub->list);

		kfree(sub);
	}
}

int v4l2_event_dequeue(struct v4l2_events *events, struct v4l2_event *event)
{
	struct _v4l2_event *ev;
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&events->lock, flags);

	if (list_empty(&events->available)) {
		spin_unlock_irqrestore(&events->lock, flags);
		ret = -ENOENT;
		goto out;
	}

	ev = list_first_entry(&events->available, struct _v4l2_event, list);
	list_del(&ev->list);

	atomic_dec(&events->navailable);
	ev->event.count = atomic_read(&events->navailable);

	spin_unlock_irqrestore(&events->lock, flags);

	memcpy(event, &ev->event, sizeof(ev->event));

	kmem_cache_free(event_kmem, ev);

out:
	return ret;
}
EXPORT_SYMBOL_GPL(v4l2_event_dequeue);

static struct v4l2_subscribed_event *__v4l2_event_subscribed(
	struct v4l2_events *events, u32 type)
{
	struct v4l2_subscribed_event *ev;

	list_for_each_entry(ev, &events->subscribed, list) {
		if (ev->type == type)
			return ev;
	}

	return NULL;
}

struct v4l2_subscribed_event *v4l2_event_subscribed(
	struct v4l2_events *events, u32 type)
{
	struct v4l2_subscribed_event *ev;
	unsigned long flags;

	spin_lock_irqsave(&events->lock, flags);

	ev = __v4l2_event_subscribed(events, type);

	spin_unlock_irqrestore(&events->lock, flags);

	return ev;
}
EXPORT_SYMBOL_GPL(v4l2_event_subscribed);

void v4l2_event_queue(struct video_device *vdev, struct v4l2_event *ev)
{
	struct v4l2_fh *fh;
	unsigned long flags;

	spin_lock_irqsave(&vdev->fh_lock, flags);

	list_for_each_entry(fh, &vdev->fh, list) {
		struct _v4l2_event *_ev;

		if (atomic_read(&fh->events.navailable) >= V4L2_MAX_EVENTS)
			continue;

		if (!v4l2_event_subscribed(&fh->events, ev->type))
			continue;

		_ev = kmem_cache_alloc(event_kmem, GFP_ATOMIC);
		if (!_ev)
			continue;

		_ev->event = *ev;

		spin_lock(&fh->events.lock);
		_ev->event.sequence = fh->events.sequence;
		fh->events.sequence++;
		list_add_tail(&_ev->list, &fh->events.available);
		spin_unlock(&fh->events.lock);

		atomic_inc(&fh->events.navailable);

		wake_up_all(&fh->events.wait);
	}

	spin_unlock_irqrestore(&vdev->fh_lock, flags);
}
EXPORT_SYMBOL_GPL(v4l2_event_queue);

int v4l2_event_pending(struct v4l2_events *events)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&events->lock, flags);
	ret = !list_empty(&events->available);
	spin_unlock_irqrestore(&events->lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(v4l2_event_pending);

int v4l2_event_subscribe(struct v4l2_events *events,
			 struct v4l2_event_subscription *sub)
{
	int ret = 0;
	unsigned long flags;
	struct v4l2_subscribed_event *ev;

	ev = kmalloc(sizeof(*ev), GFP_KERNEL);
	if (!ev)
		return -ENOMEM;

	spin_lock_irqsave(&events->lock, flags);

	if (__v4l2_event_subscribed(events, sub->type) != NULL) {
		ret = -EBUSY;
		goto out;
	}

	INIT_LIST_HEAD(&ev->list);
	ev->type = sub->type;

	list_add(&ev->list, &events->subscribed);

out:
	spin_unlock_irqrestore(&events->lock, flags);

	if (ret)
		kfree(ev);

	return ret;
}
EXPORT_SYMBOL_GPL(v4l2_event_subscribe);

int v4l2_event_unsubscribe(struct v4l2_events *events,
			   struct v4l2_event_subscription *sub)
{
	struct v4l2_subscribed_event *ev;
	unsigned long flags;

	spin_lock_irqsave(&events->lock, flags);

	ev = __v4l2_event_subscribed(events, sub->type);

	if (ev != NULL)
		list_del(&ev->list);

	spin_unlock_irqrestore(&events->lock, flags);

	return ev == NULL;
}
EXPORT_SYMBOL_GPL(v4l2_event_unsubscribe);
