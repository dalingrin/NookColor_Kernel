/*
 * drv_interface.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * DSP/BIOS Bridge driver interface.
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

/*  ----------------------------------- Host OS */

#include <dspbridge/host_os.h>
#include <linux/platform_device.h>
#include <linux/pm.h>

#ifdef MODULE
#include <linux/module.h>
#endif

#include <linux/device.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/cdev.h>
#include <linux/kobject.h>

#include <mach/board-3430sdp.h>

/*  ----------------------------------- DSP/BIOS Bridge */
#include <dspbridge/std.h>
#include <dspbridge/dbdefs.h>
#include <dspbridge/errbase.h>
#include <_tiomap.h>

/*  ----------------------------------- Trace & Debug */
#include <dspbridge/gt.h>
#include <dspbridge/dbc.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <dspbridge/services.h>
#include <dspbridge/sync.h>
#include <dspbridge/reg.h>

/*  ----------------------------------- Platform Manager */
#include <dspbridge/wcdioctl.h>
#include <dspbridge/_dcd.h>
#include <dspbridge/dspdrv.h>
#ifdef CONFIG_BRIDGE_WDT3
#include <dspbridge/clk.h>
#include <dspbridge/io_sm.h>
#include <_tiomap.h>
#endif

/*  ----------------------------------- Resource Manager */
#include <dspbridge/pwr.h>

/*  ----------------------------------- This */
#include <drv_interface.h>

#include <dspbridge/cfg.h>
#include <dspbridge/resourcecleanup.h>
#include <dspbridge/chnl.h>
#include <dspbridge/proc.h>
#include <dspbridge/dev.h>
#include <dspbridge/drvdefs.h>
#include <dspbridge/drv.h>

#include <mach/omap-pm.h>

#define BRIDGE_NAME "C6410"
/*  ----------------------------------- Globals */
#define DRIVER_NAME  "DspBridge"
s32 dsp_debug;

struct platform_device *omap_dspbridge_dev;


static struct cdev bridge_cdev;

static struct class *bridge_class;

static u32 driverContext;
#ifdef CONFIG_BRIDGE_DEBUG
static char *GT_str;
#endif /* CONFIG_BRIDGE_DEBUG */
static s32 driver_major;
static char *base_img;
char *iva_img;
static s32 shm_size = 0x500000;	/* 5 MB */
static u32 phys_mempool_base;
static u32 phys_mempool_size;
static int tc_wordswapon;	/* Default value is always false */
#ifdef CONFIG_BRIDGE_RECOVERY
static atomic_t bridge_cref;	/* number of bridge open handles */
static struct workqueue_struct *bridge_rec_queue;
static struct work_struct bridge_recovery_work;
static DECLARE_COMPLETION_ONSTACK(bridge_comp);
static DECLARE_COMPLETION_ONSTACK(bridge_open_comp);
static bool recover;
static char *firmware_file = "/system/lib/dsp/baseimage.dof";
#endif

/* Minimum ACTIVE VDD1 OPP level for reliable DSP operation */
unsigned long min_dsp_freq;

#ifdef CONFIG_PM
struct omap34xx_bridge_suspend_data {
	int suspended;
	wait_queue_head_t suspend_wq;
};

static struct omap34xx_bridge_suspend_data bridge_suspend_data;

static void bridge_create_sysfs(void);
static void bridge_destroy_sysfs(void);

static int omap34xxbridge_suspend_lockout(
		struct omap34xx_bridge_suspend_data *s, struct file *f)
{
	if ((s)->suspended) {
		if ((f)->f_flags & O_NONBLOCK)
			return DSP_EDPMSUSPEND;
		wait_event_interruptible((s)->suspend_wq, (s)->suspended == 0);
	}
	return 0;
}

#endif

#ifdef CONFIG_BRIDGE_DEBUG
module_param(GT_str, charp, 0);
MODULE_PARM_DESC(GT_str, "GT string, default = NULL");

module_param(dsp_debug, int, 0);
MODULE_PARM_DESC(dsp_debug, "Wait after loading DSP image. default = false");
#endif
#ifdef CONFIG_BRIDGE_RECOVERY
module_param(firmware_file, charp, 0644);
#endif

module_param(base_img, charp, 0);
MODULE_PARM_DESC(base_img, "DSP base image, default = NULL");

module_param(shm_size, int, 0);
MODULE_PARM_DESC(shm_size, "SHM size, default = 4 MB, minimum = 64 KB");

module_param(phys_mempool_base, uint, 0);
MODULE_PARM_DESC(phys_mempool_base,
		"Physical memory pool base passed to driver");

module_param(phys_mempool_size, uint, 0);
MODULE_PARM_DESC(phys_mempool_size,
		"Physical memory pool size passed to driver");
module_param(tc_wordswapon, int, 0);
MODULE_PARM_DESC(tc_wordswapon, "TC Word Swap Option. default = 0");

module_param(min_dsp_freq, ulong, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(min_dsp_freq, "Minimum ACTIVE VDD1 OPP Level, default = 1");

MODULE_AUTHOR("Texas Instruments");
MODULE_LICENSE("GPL");

static char *driver_name = DRIVER_NAME;

#ifdef CONFIG_BRIDGE_DEBUG
static struct GT_Mask driverTrace;
#endif /* CONFIG_BRIDGE_DEBUG */

static const struct file_operations bridge_fops = {
	.open		= bridge_open,
	.release	= bridge_release,
	.unlocked_ioctl	= bridge_ioctl,
	.mmap		= bridge_mmap,
};

#ifdef CONFIG_PM
static u32 timeOut = 1000;
#ifdef CONFIG_BRIDGE_DVFS
static struct clk *clk_handle;
#endif
#endif

struct dspbridge_platform_data *omap_dspbridge_pdata;

#ifdef CONFIG_BRIDGE_RECOVERY
static void bridge_load_firmware(void)
{
	struct PROCESS_CONTEXT pr_ctxt;
	DSP_STATUS status;
	const char *argv[2];
	argv[0] = firmware_file;
	argv[1] = NULL;

	pr_ctxt.hProcessor = NULL;
	status = PROC_Attach(0, NULL, &pr_ctxt.hProcessor, &pr_ctxt);
	if (DSP_FAILED(status))
		goto func_err;

	status = PROC_Stop(pr_ctxt.hProcessor);
	if (DSP_FAILED(status))
		goto func_err;

	status = PROC_Load(pr_ctxt.hProcessor, 1, argv, NULL);
	if (DSP_FAILED(status))
		goto func_err;

	status = PROC_Start(pr_ctxt.hProcessor);
	if (DSP_FAILED(status))
		goto func_err;

	status = PROC_Detach(&pr_ctxt);
	if (DSP_SUCCEEDED(status)) {
		pr_info("DSP recovery succeeded\n");
		return;
	}

func_err:
	pr_err("DSP could not be restarted, status = %x\n", status);
}

static void bridge_recover(struct work_struct *work)
{
	if (atomic_read(&bridge_cref)) {
		INIT_COMPLETION(bridge_comp);
		wait_for_completion(&bridge_comp);
	}
	bridge_load_firmware();
	recover = false;
	complete_all(&bridge_open_comp);
}

void bridge_recover_schedule(void)
{
	INIT_COMPLETION(bridge_open_comp);
	recover = true;
	pr_err("DSP crash, attempting to reset\n");
	queue_work(bridge_rec_queue, &bridge_recovery_work);
}
#endif

#ifdef CONFIG_BRIDGE_DVFS
static int dspbridge_post_scale(struct notifier_block *op, unsigned long level,
				void *ptr)
{
	PWR_PM_PostScale(PRCM_VDD1, level);
	return 0;
}

static struct notifier_block iva_clk_notifier = {
	.notifier_call = dspbridge_post_scale,
	NULL,
};
#endif

static int __devinit omap34xx_bridge_probe(struct platform_device *pdev)
{
	int status;
	u32 initStatus;
	u32 temp;
	dev_t   dev = 0 ;
	int     result;
	struct dspbridge_platform_data *pdata = pdev->dev.platform_data;

	omap_dspbridge_dev = pdev;

	/* use 2.6 device model */
	result = alloc_chrdev_region(&dev, 0, 1, driver_name);
	if (result < 0) {
		pr_err("%s: Can't get major %d\n", __func__, driver_major);
		goto err1;
	}

	driver_major = MAJOR(dev);

	cdev_init(&bridge_cdev, &bridge_fops);
	bridge_cdev.owner = THIS_MODULE;

	status = cdev_add(&bridge_cdev, dev, 1);
	if (status) {
		pr_err("%s: Failed to add bridge device\n", __func__);
		goto err2;
	}

	/* udev support */
	bridge_class = class_create(THIS_MODULE, "ti_bridge");

	if (IS_ERR(bridge_class))
		pr_err("%s: Error creating bridge class\n", __func__);

	device_create(bridge_class, NULL, MKDEV(driver_major, 0),
			NULL, "DspBridge");

	bridge_create_sysfs();

	GT_init();
	GT_create(&driverTrace, "LD");

#ifdef CONFIG_BRIDGE_DEBUG
	if (GT_str)
		GT_set(GT_str);
#elif defined(DDSP_DEBUG_PRODUCT) && GT_TRACE
	GT_set("**=67");
#endif

#ifdef CONFIG_PM
	/* Initialize the wait queue */
	if (!status) {
		bridge_suspend_data.suspended = 0;
		init_waitqueue_head(&bridge_suspend_data.suspend_wq);
	}
#endif

	SERVICES_Init();

	/*  Autostart flag.  This should be set to true if the DSP image should
	 *  be loaded and run during bridge module initialization  */

	if (base_img) {
		temp = true;
		REG_SetValue(AUTOSTART, (u8 *)&temp, sizeof(temp));
		REG_SetValue(DEFEXEC, (u8 *)base_img, strlen(base_img) + 1);
	} else {
		temp = false;
		REG_SetValue(AUTOSTART, (u8 *)&temp, sizeof(temp));
		REG_SetValue(DEFEXEC, (u8 *) "\0", (u32)2);
	}

	if (shm_size >= 0x10000) {	/* 64 KB */
		initStatus = REG_SetValue(SHMSIZE, (u8 *)&shm_size,
				sizeof(shm_size));
	} else {
		initStatus = DSP_EINVALIDARG;
		status = -1;
		pr_err("%s: SHM size must be at least 64 KB\n", __func__);
	}
	GT_1trace(driverTrace, GT_7CLASS,
		 "requested shm_size = 0x%x\n", shm_size);

	if (pdata->phys_mempool_base && pdata->phys_mempool_size) {
		phys_mempool_base = pdata->phys_mempool_base;
		phys_mempool_size = pdata->phys_mempool_size;
	}

	GT_1trace(driverTrace, GT_7CLASS, "phys_mempool_base = 0x%x \n",
		 phys_mempool_base);

	GT_1trace(driverTrace, GT_7CLASS, "phys_mempool_size = 0x%x\n",
		 phys_mempool_base);

	if ((phys_mempool_base > 0x0) && (phys_mempool_size > 0x0))
		MEM_ExtPhysPoolInit(phys_mempool_base, phys_mempool_size);
	if (tc_wordswapon) {
		GT_0trace(driverTrace, GT_7CLASS, "TC Word Swap is enabled\n");
		REG_SetValue(TCWORDSWAP, (u8 *)&tc_wordswapon,
				sizeof(tc_wordswapon));
	} else {
		GT_0trace(driverTrace, GT_7CLASS, "TC Word Swap is disabled\n");
		REG_SetValue(TCWORDSWAP, (u8 *)&tc_wordswapon,
				sizeof(tc_wordswapon));
	}
	if (DSP_SUCCEEDED(initStatus)) {
#ifdef CONFIG_BRIDGE_DVFS
		clk_handle = clk_get(NULL, "iva2_ck");
		if (!clk_handle)
			pr_err("%s: clk_get failed to get iva2_ck\n", __func__);

		if (clk_notifier_register(clk_handle, &iva_clk_notifier))
			pr_err("%s: clk_notifier_register failed for iva2_ck\n",
								__func__);

		if (!min_dsp_freq)
			min_dsp_freq = pdata->mpu_min_speed;
#endif
		driverContext = DSP_Init(&initStatus);
		if (DSP_FAILED(initStatus)) {
			status = -1;
			pr_err("DSP Bridge driver initialization failed\n");
		} else {
			pr_info("DSP Bridge driver loaded\n");
		}
	}
#ifdef CONFIG_BRIDGE_RECOVERY
	bridge_rec_queue = create_workqueue("bridge_rec_queue");
	INIT_WORK(&bridge_recovery_work, bridge_recover);
	INIT_COMPLETION(bridge_comp);
#endif

	DBC_Assert(status == 0);
	DBC_Assert(DSP_SUCCEEDED(initStatus));

	return 0;

err2:
	unregister_chrdev_region(dev, 1);
err1:
	return result;
}

static int __devexit omap34xx_bridge_remove(struct platform_device *pdev)
{
	dev_t devno;
	bool ret;
	DSP_STATUS dsp_status = DSP_SOK;
	HANDLE hDrvObject = NULL;

	dsp_status = CFG_GetObject((u32 *)&hDrvObject, REG_DRV_OBJECT);
	if (DSP_FAILED(dsp_status))
		goto func_cont;

#ifdef CONFIG_BRIDGE_DVFS
	if (clk_notifier_unregister(clk_handle, &iva_clk_notifier))
		pr_err("%s: clk_notifier_unregister failed for iva2_ck\n",
								__func__);
#endif /* #ifdef CONFIG_BRIDGE_DVFS */

	if (driverContext) {
		/* Put the DSP in reset state */
		ret = DSP_Deinit(driverContext);
		driverContext = 0;
		DBC_Assert(ret == true);
	}

#ifdef CONFIG_BRIDGE_DVFS
	clk_put(clk_handle);
	clk_handle = NULL;
#endif

func_cont:
	MEM_ExtPhysPoolRelease();

	SERVICES_Exit();
	GT_exit();

	/* Remove driver sysfs entries */
	bridge_destroy_sysfs();

	devno = MKDEV(driver_major, 0);
	cdev_del(&bridge_cdev);
	unregister_chrdev_region(devno, 1);
	if (bridge_class) {
		/* remove the device from sysfs */
		device_destroy(bridge_class, MKDEV(driver_major, 0));
		class_destroy(bridge_class);

	}
	return 0;
}


#ifdef CONFIG_PM
static int bridge_suspend(struct platform_device *pdev, pm_message_t state)
{
	u32 status;
	u32 command = PWR_EMERGENCYDEEPSLEEP;

	status = PWR_SleepDSP(command, timeOut);
	if (DSP_FAILED(status))
		return -1;

	bridge_suspend_data.suspended = 1;
	return 0;
}

static int bridge_resume(struct platform_device *pdev)
{
	u32 status;

	status = PWR_WakeDSP(timeOut);
	if (DSP_FAILED(status))
		return -1;

	bridge_suspend_data.suspended = 0;
	wake_up(&bridge_suspend_data.suspend_wq);
	return 0;
}
#else
#define bridge_suspend NULL
#define bridge_resume NULL
#endif

static struct platform_driver bridge_driver = {
	.driver = {
		.name = BRIDGE_NAME,
	},
	.probe	 = omap34xx_bridge_probe,
	.remove	 = __devexit_p(omap34xx_bridge_remove),
	.suspend = bridge_suspend,
	.resume	 = bridge_resume,
};

static int __init bridge_init(void)
{
	return platform_driver_register(&bridge_driver);
}

static void __exit bridge_exit(void)
{
	platform_driver_unregister(&bridge_driver);
}

/* This function is called when an application opens handle to the
 * bridge driver. */
static int bridge_open(struct inode *ip, struct file *filp)
{
	int status = 0;
	struct PROCESS_CONTEXT *pr_ctxt = NULL;

#ifdef CONFIG_BRIDGE_RECOVERY
	if (recover)
		wait_for_completion(&bridge_open_comp);
#endif

	/*
	 * Allocate a new process context and insert it into global
	 * process context list.
	 */
	pr_ctxt = MEM_Calloc(sizeof(struct PROCESS_CONTEXT), MEM_PAGED);
	if (pr_ctxt) {
		pr_ctxt->resState = PROC_RES_ALLOCATED;
		spin_lock_init(&pr_ctxt->dmm_map_lock);
		INIT_LIST_HEAD(&pr_ctxt->dmm_map_list);
		spin_lock_init(&pr_ctxt->dmm_rsv_lock);
		INIT_LIST_HEAD(&pr_ctxt->dmm_rsv_list);
		mutex_init(&pr_ctxt->node_mutex);
		mutex_init(&pr_ctxt->strm_mutex);
	} else {
		status = -ENOMEM;
	}

		filp->private_data = pr_ctxt;

#ifdef CONFIG_BRIDGE_RECOVERY
	if (!status)
		atomic_inc(&bridge_cref);
#endif

	return status;
}

/* This function is called when an application closes handle to the bridge
 * driver. */
static int bridge_release(struct inode *ip, struct file *filp)
{
	struct PROCESS_CONTEXT *pr_ctxt;
	int status = 0;

	if (!filp->private_data) {
		status = -EIO;
		goto err;
	}
	pr_ctxt = filp->private_data;

	if (pr_ctxt) {
		flush_signals(current);
		DRV_RemoveAllResources(pr_ctxt);
		if (pr_ctxt->hProcessor)
			PROC_Detach(pr_ctxt);
		kfree(pr_ctxt);
		filp->private_data = NULL;
	}
err:
#ifdef CONFIG_BRIDGE_RECOVERY
	if (!atomic_dec_return(&bridge_cref))
		complete(&bridge_comp);
#endif

	return status;
}

/* This function provides IO interface to the bridge driver. */
static long bridge_ioctl(struct file *filp, unsigned int code,
		unsigned long args)
{
	int status;
	u32 retval = DSP_SOK;
	union Trapped_Args pBufIn;

	DBC_Require(filp != NULL);
#ifdef CONFIG_BRIDGE_RECOVERY
	if (recover) {
		status = -EBUSY;
		goto err;
	}
#endif

#ifdef CONFIG_PM
	status = omap34xxbridge_suspend_lockout(&bridge_suspend_data, filp);
	if (status != 0)
		return status;
#endif

	/* Deduct one for the CMD_BASE. */
	code = (code - 1);
	if (!filp->private_data) {
		status = -EIO;
		goto err;
	}

	status = copy_from_user(&pBufIn, (union Trapped_Args *)args,
				sizeof(union Trapped_Args));

	if (!status) {
		status = WCD_CallDevIOCtl(code, &pBufIn, &retval,
				filp->private_data);

		if (DSP_SUCCEEDED(status)) {
			status = retval;
		} else {
			GT_1trace(driverTrace, GT_7CLASS,
				 "IOCTL Failed, code : 0x%x\n", code);
			status = -1;
		}

	}

err:
	return status;
}

/* This function maps kernel space memory to user space memory. */
static int bridge_mmap(struct file *filp, struct vm_area_struct *vma)
{
#if GT_TRACE
	u32 offset = vma->vm_pgoff << PAGE_SHIFT;
#endif
	u32 status;

	DBC_Assert(vma->vm_start < vma->vm_end);

	vma->vm_flags |= VM_RESERVED | VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	GT_6trace(driverTrace, GT_3CLASS,
		 "vm filp %p offset %lx start %lx end %lx"
		 " page_prot %lx flags %lx\n", filp, offset, vma->vm_start,
		 vma->vm_end, vma->vm_page_prot, vma->vm_flags);

	status = remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
				vma->vm_end - vma->vm_start, vma->vm_page_prot);
	if (status != 0)
		status = -EAGAIN;

	return status;
}

/* To remove all process resources before removing the process from the
 * process context list*/
DSP_STATUS DRV_RemoveAllResources(HANDLE hPCtxt)
{
	DSP_STATUS status = DSP_SOK;
	struct PROCESS_CONTEXT *pCtxt = (struct PROCESS_CONTEXT *)hPCtxt;
	DRV_RemoveAllSTRMResElements(pCtxt);
	DRV_RemoveAllNodeResElements(pCtxt);
	DRV_RemoveAllDMMResElements(pCtxt);
	pCtxt->resState = PROC_RES_FREED;
	return status;
}

/*
 * sysfs
 */
static ssize_t drv_state_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct WMD_DEV_CONTEXT *dwContext;
	struct DEV_OBJECT *hDevObject = NULL;
	int drv_state = 0;

	for (hDevObject = (struct DEV_OBJECT *)DRV_GetFirstDevObject();
		hDevObject != NULL;
		hDevObject = (struct DEV_OBJECT *)DRV_GetNextDevObject
							((u32)hDevObject)) {
		DEV_GetWMDContext(hDevObject, &dwContext);
		if (!dwContext)
			continue;

		drv_state = dwContext->dwBrdState;
	}

	return sprintf(buf, "%d\n", drv_state);
}

/*
 * this sysfs is intended to retrieve two MPU addresses
 * needed for the INST2 utility.
 * the inst_log script will run this sysfs
 */
static ssize_t mpu_address_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
    struct WMD_DEV_CONTEXT *dwContext = NULL;
	struct DEV_OBJECT *hDevObject = NULL;
	u32 mem_poolsize = 0;
	u32 GppPa = 0, DspVa = 0;
	u32 armPhyMemOffUncached = 0;
	hDevObject = (struct DEV_OBJECT *)DRV_GetFirstDevObject();
	DEV_GetWMDContext(hDevObject, &dwContext);
	if (!dwContext)
		return 0;
	GppPa = dwContext->aTLBEntry[0].ulGppPa;
	DspVa = dwContext->aTLBEntry[0].ulDspVa;

	/*
	 * the physical address offset, this offset is a
	 * fixed value for a given platform.
	 */
	armPhyMemOffUncached = GppPa - DspVa;

	/*
	 * the offset value for cached address region
	 * on DSP address space
	 */
	mem_poolsize = phys_mempool_base - 0x20000000;

	/* Retrive the above calculated addresses */
	return sprintf(buf, "mempoolsizeOffset 0x%x GppPaOffset 0x%x\n",
					mem_poolsize, armPhyMemOffUncached);
}

static DEVICE_ATTR(drv_state, S_IRUGO, drv_state_show, NULL);
static DEVICE_ATTR(mpu_address, S_IRUGO, mpu_address_show, NULL);

#ifdef CONFIG_BRIDGE_WDT3
static ssize_t wdt3_show(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	return sprintf(buf, "%d\n", (dsp_wdt_get_enable()) ? 1 : 0);
}

static ssize_t wdt3_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t n)
{
	u32 wdt3;
	struct DEV_OBJECT *dev_object;
	struct WMD_DEV_CONTEXT *dev_ctxt;

	if (sscanf(buf, "%d", &wdt3) != 1)
		return -EINVAL;

	dev_object = DEV_GetFirst();
	if (dev_object == NULL)
		goto func_end;
	DEV_GetWMDContext(dev_object, &dev_ctxt);
	if (dev_ctxt == NULL)
		goto func_end;

	/* enable WDT */
	if (wdt3 == 1) {
		if (dsp_wdt_get_enable())
			goto func_end;
		dsp_wdt_set_enable(true);
		if (!CLK_Get_UseCnt(SERVICESCLK_wdt3_fck) &&
				dev_ctxt->dwBrdState != BRD_DSP_HIBERNATION)
			dsp_wdt_enable(true);
	} else if (wdt3 == 0) {
		if (!dsp_wdt_get_enable())
			goto func_end;
		if (CLK_Get_UseCnt(SERVICESCLK_wdt3_fck))
			dsp_wdt_enable(false);
		dsp_wdt_set_enable(false);
	}
func_end:
	return n;
}

static DEVICE_ATTR(dsp_wdt, S_IWUSR | S_IRUGO, wdt3_show, wdt3_store);

static ssize_t wdt3_timeout_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", dsp_wdt_get_timeout());
}

static ssize_t wdt3_timeout_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	u32 wdt3_to;

	if (sscanf(buf, "%d", &wdt3_to) != 1)
		return -EINVAL;

	dsp_wdt_set_timeout(wdt3_to);
	return n;
}

static DEVICE_ATTR(dsp_wdt_timeout, S_IWUSR | S_IRUGO, wdt3_timeout_show,
			wdt3_timeout_store);
#endif

static struct attribute *attrs[] = {
	&dev_attr_drv_state.attr,
	&dev_attr_mpu_address.attr,
#ifdef CONFIG_BRIDGE_WDT3
	&dev_attr_dsp_wdt.attr,
	&dev_attr_dsp_wdt_timeout.attr,
#endif
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static void bridge_create_sysfs(void)
{
	int error;

	error = sysfs_create_group(&omap_dspbridge_dev->dev.kobj, &attr_group);

	if (error)
		kobject_put(&omap_dspbridge_dev->dev.kobj);
}

static void bridge_destroy_sysfs(void)
{
	sysfs_remove_group(&omap_dspbridge_dev->dev.kobj, &attr_group);
}

/* Bridge driver initialization and de-initialization functions */
module_init(bridge_init);
module_exit(bridge_exit);

