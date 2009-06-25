/*
 * board-mapphone-gpio.c
 *
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* Date	 Author	  Comment
 * ===========  ==============  ==============================================
 * Jun-23-2009  Motorola	Initial revision.
 */

#include <linux/module.h>

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#ifdef CONFIG_GPIO_MAPPING
#include <linux/gpio_mapping.h>
#endif
#endif

#if defined(CONFIG_ARM_OF) && defined(CONFIG_GPIO_MAPPING)
struct omap_gpio_map_entry {
    u32 pin_num;
    char name[GPIO_MAP_NAME_SIZE];
} __attribute__ ((__packed__));

void trim_gpio_map_string(char *s)
{
	int i;

	/* ignore all characters behind space key */
	for (i = 0; i < GPIO_MAP_NAME_SIZE; i++) {
		if (' ' == s[i]) {
			s[i] = '\0';
			return;
		}
	}

    printk(KERN_ERR "Too long gpio map string name!\n");
}
#endif

void __init mapphone_gpio_mapping_init(void)
{
#if defined(CONFIG_ARM_OF) && defined(CONFIG_GPIO_MAPPING)
	struct device_node *node;
	const void *prop;
	int i, j, size, unit_size;
	char name[GPIO_MAP_NAME_SIZE];

	node = of_find_node_by_path(DT_PATH_GPIO);
	if (node == NULL) {
		printk(KERN_ERR
				"Unable to read node %s from device tree!\n",
				DT_PATH_GPIO);
		return;
	}

	unit_size = sizeof(struct omap_gpio_map_entry);
	prop = of_get_property(node, DT_PROP_GPIO_MAP, &size);
	if ((!prop) || (size % unit_size)) {
		printk(KERN_ERR "Read property %s error!\n",
				DT_PROP_GPIO_MAP);
		of_node_put(node);
		return;
	}

	for (i = 0; i < size / unit_size; i++) {
		struct omap_gpio_map_entry *p =
				(struct omap_gpio_map_entry *) prop;

		memcpy((void *) name, p->name, GPIO_MAP_NAME_SIZE);
		trim_gpio_map_string(name);

		for (j = 0; j < GPIO_MAP_SIZE; j++) {
			if (gpio_map_table[j].used == 0) {
				gpio_map_table[j].used = 1;
				gpio_map_table[j].pin_num = p->pin_num;
				strncpy(gpio_map_table[j].name, name,
						GPIO_MAP_NAME_SIZE);
				break;
			} else if (strncmp(gpio_map_table[j].name, name,
					GPIO_MAP_NAME_SIZE) == 0) {
				gpio_map_table[j].pin_num = p->pin_num;
				break;
			}
		}

		if (j == GPIO_MAP_SIZE)
			printk(KERN_ERR "Unable to write gpio_map_table\n");
		else
			printk(KERN_INFO "GPIO mapping write: pin = %d, name = %s\n",
						gpio_map_table[j].pin_num,
						gpio_map_table[j].name);

		prop += unit_size;
	}

    of_node_put(node);
    printk(KERN_INFO "GPIO mapping init done!\n");
#else
	printk(KERN_INFO "GPIO Mapping: Using no-dt configuration!\n");
#endif
}
