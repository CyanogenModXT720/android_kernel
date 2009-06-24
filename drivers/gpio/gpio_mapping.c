/*
 *                                        gpio_mapping.c
 *
 * Copyright (C) 2009 Motorola, Inc.
 */

/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

/* Date         Author      Comment
 * ===========  ==========  ==================================================
 * 10-Jun-2009  Motorola    Initial.
 */

#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/gpio_mapping.h>

struct gpio_mapping gpio_map_table[GPIO_MAP_SIZE] = {
	{1, 92, "lcd_panel_reset"},
	{1, 93, "lcd_panel_sd"},
	{1, 149, "usb_ipc_phy_reset"},
	{1, 164, "touch_panel_reset"},
	{1, 163, "mmc_detect"},
	{1, 177, "slider_data"},
	{1, 65, "wlan_host_wake"},
};

int get_gpio_by_name(char *name)
{
	int i;

	for (i = 0; i < GPIO_MAP_SIZE; i++) {
		if (gpio_map_table[i].used == 0)
			continue;

		if (strncmp(name, gpio_map_table[i].name, GPIO_MAP_NAME_SIZE)
			== 0)
			return gpio_map_table[i].pin_num;
	}

	printk(KERN_ERR "Unable to get gpio pin num for %s\n", name);
	return -EINVAL;
}
EXPORT_SYMBOL(get_gpio_by_name);
