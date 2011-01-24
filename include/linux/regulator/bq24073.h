/*
 * Support for TI bq24073 (bqTINY-II) Dual Input (USB/AC Adpater)
 * 1-Cell Li-Ion Charger connected via GPIOs.
 *
 * Copyright (c) 2008 Philipp Zabel
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

 struct regulator_init_data;

/**
 * bq24073_mach_info - platform data for bq24073
 * @gpio_nce: GPIO line connected to the nCE pin, used to enable / disable
 * charging
 * @gpio_iset2: GPIO line connected to the ISET2 pin, used to limit charging
 * current to 100 mA / 500 mA
 */
struct bq24073_mach_info {
	int gpio_nce;
	int gpio_en1;
	int gpio_en2;
	int gpio_nce_state;
	int gpio_en1_state;
	int gpio_en2_state;
	struct regulator_init_data *init_data;
};
