/*
 * drivers/media/video/v4l2-fh.c
 *
 * V4L2 file handles.
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
#include <media/v4l2-fh.h>

#include <linux/sched.h>
#include <linux/vmalloc.h>

int v4l2_fh_add(struct video_device *vdev, struct v4l2_fh *fh)
{
	unsigned long flags;

	v4l2_event_init_fh(&fh->events);

	spin_lock_irqsave(&vdev->fh_lock, flags);
	list_add(&fh->list, &vdev->fh);
	spin_unlock_irqrestore(&vdev->fh_lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(v4l2_fh_add);

void v4l2_fh_del(struct video_device *vdev, struct v4l2_fh *fh)
{
	unsigned long flags;

	v4l2_event_del(&fh->events);

	spin_lock_irqsave(&vdev->fh_lock, flags);
	list_del(&fh->list);
	spin_unlock_irqrestore(&vdev->fh_lock, flags);
}
EXPORT_SYMBOL_GPL(v4l2_fh_del);

int v4l2_fh_init(struct video_device *vdev)
{
	spin_lock_init(&vdev->fh_lock);
	INIT_LIST_HEAD(&vdev->fh);

	return 0;
}

void v4l2_fh_exit(struct video_device *vdev)
{
}

