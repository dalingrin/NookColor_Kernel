#ifndef __OMAP3_OPP_H_
#define __OMAP3_OPP_H_

#include <mach/omap-pm.h>

static struct omap_opp omap3630_mpu_rate_table[] = {
	{0, 0, 0},
	/*OPP1 (OPP50) - 1.0V*/
	{S300M, VDD1_OPP1, 0x20, 0x0, 0x0},
	/*OPP2 (OPP100) - 1.1625V*/
	{S600M, VDD1_OPP2, 0x2d, 0x0, 0x0},
	/*OPP3 (OPP130) - 1.3V*/
	{S800M, VDD1_OPP3, 0x38, 0x0, 0x0},
	/*OPP4 (OPP-1G) - 1.35V*/
	{S1000M, VDD1_OPP4, 0x3c, 0x0, 0x0},
	/*OPP5 (OPP-1.3G) - 1.35V*/
	{S1100M, VDD1_OPP5, 0x3c, 0x0, 0x0},
};

static struct omap_opp omap3630_l3_rate_table[] = {
	{0, 0, 0},
	/*OPP1 (OPP50) - 0.97V*/
	{S100M, VDD2_OPP1, 0x1e, 0x0, 0x0},
	/*OPP2 (OPP100) - 1.1625V*/
	{S200M, VDD2_OPP2, 0x2d, 0x0, 0x0},
};

static struct omap_opp omap3630_dsp_rate_table[] = {
	{0, 0, 0},
	/*OPP1 (OPP50) - 1.0V*/
	{S260M, VDD1_OPP1, 0x20, 0x0, 0x0},
	/*OPP2 (OPP100) - 1.1625V*/
	{S520M, VDD1_OPP2, 0x2d, 0x0, 0x0},
	/*OPP3 (OPP130) - 1.3V*/
	{S660M, VDD1_OPP3, 0x38, 0x0, 0x0},
	/*OPP4 (OPP-1G) - 1.35V*/
	{S800M, VDD1_OPP4, 0x3c, 0x0, 0x0},
	/*OPP5 (OPP-1.3G) - 1.35V*/
	{S65M, VDD1_OPP5, 0x3c, 0x0, 0x0},
};

static struct omap_opp omap3621_mpu_rate_table[] = {
	{0, 0, 0},
	/*OPP1 (OPP50) - 0.93mV*/
	{S300M, VDD1_OPP1, 0x20, 0x0},
	/*OPP2 (OPP100) - 1.1V*/
	{S600M, VDD1_OPP2, 0x2d, 0x0},
	/*OPP3 (OPP130) - 1.26V*/
	{S800M, VDD1_OPP3, 0x38, 0x0},
	/*OPP4 (OPP-1G) - 1.35V*/
	{S800M, VDD1_OPP4, 0x3c, 0x0},
	/*OPP5 (OPP-1G) - 1.35V*/
	{S800M, VDD1_OPP5, 0x3c, 0x0},
};

static struct omap_opp omap3621_l3_rate_table[] = {
	{0, 0, 0},
	/*OPP1 (OPP50) - 0.93V*/
	{S83M, VDD2_OPP1, 0x1e, 0x0},
	/*OPP2 (OPP100) - 1.1375V*/
	{S166M, VDD2_OPP2, 0x2d, 0x0},
};

static struct omap_opp omap3621_dsp_rate_table[] = {
	{0, 0, 0},
	/*OPP1 (OPP50) - 0.93V*/
	{S260M, VDD1_OPP1, 0x20, 0x0},
	/*OPP2 (OPP100) - 1.1V*/
	{S520M, VDD1_OPP2, 0x2d, 0x0},
	/*OPP3 (OPP130) - 1.26V*/
	{S660M, VDD1_OPP3, 0x38, 0x0},
	/*OPP4 (OPP-1G) - 1.35V*/
	{S660M, VDD1_OPP4, 0x3c, 0x0},
	/*OPP5 (OPP-1G) - 1.35V*/
	{S660M, VDD1_OPP5, 0x3c, 0x0},
};

static struct omap_opp omap3_mpu_rate_table[] = {
	{0, 0, 0},
	/*OPP1*/
	{S125M, VDD1_OPP1, 0x1E},
	/*OPP2*/
	{S250M, VDD1_OPP2, 0x26},
	/*OPP3*/
	{S500M, VDD1_OPP3, 0x30},
	/*OPP4*/
	{S550M, VDD1_OPP4, 0x36},
	/*OPP5*/
	{S600M, VDD1_OPP5, 0x3C},
	/*OPP6*/
	{S720M, VDD1_OPP6, 0x3C},
};

static struct omap_opp omap3_l3_rate_table[] = {
	{0, 0, 0},
	/*OPP1*/
	{0, VDD2_OPP1, 0x1E},
	/*OPP2*/
	{S83M, VDD2_OPP2, 0x24},
	/*OPP3*/
	{S166M, VDD2_OPP3, 0x2C},
};

/* iva rate table for 3420 */
static struct omap_opp omap3_dsp_rate_table_3420[] = {
	{0, 0, 0},
	/*OPP1*/
	{S90M, VDD1_OPP1, 0x1E},
	/*OPP2*/
	{S180M, VDD1_OPP2, 0x26},
	/*OPP3*/
	{S360M, VDD1_OPP3, 0x30},
	/*OPP4*/
	{S360M, VDD1_OPP4, 0x36},
	/*OPP5*/
	{S360M, VDD1_OPP5, 0x36},
};

/* iva rate table for 3430 */
static struct omap_opp omap3_dsp_rate_table[] = {
	{0, 0, 0},
	/*OPP1*/
	{S90M, VDD1_OPP1, 0x1E},
	/*OPP2*/
	{S180M, VDD1_OPP2, 0x26},
	/*OPP3*/
	{S360M, VDD1_OPP3, 0x30},
	/*OPP4*/
	{S430M, VDD1_OPP4, 0x36},
	/*OPP5*/
	{S430M, VDD1_OPP5, 0x3C},
};

/* iva rate table for 3430 */
static struct omap_opp omap3_dsp_rate_table_3440[] = {
	{0, 0, 0},
	/*OPP1*/
	{S90M, VDD1_OPP1, 0x1E},
	/*OPP2*/
	{S180M, VDD1_OPP2, 0x26},
	/*OPP3*/
	{S360M, VDD1_OPP3, 0x30},
	/*OPP4*/
	{S430M, VDD1_OPP4, 0x36},
	/*OPP5*/
	{S430M, VDD1_OPP5, 0x3C},
	/*OPP6*/
	{S520M, VDD1_OPP6, 0x3C},
};

#endif
